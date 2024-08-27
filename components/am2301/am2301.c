#include "am2301.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// This section ensures compatibility with both ESP-IDF version 4.x and 5.x.
// In ESP-IDF 5.x, `esp_rom_delay_us` replaces the older `ets_delay_us` function.
// The `__has_include` directive is used to check which header file is available,
// allowing the code to select the appropriate delay function based on the ESP-IDF version.
#if __has_include("esp_rom_sys.h")
    #include "esp_rom_sys.h"
    #define delay_us esp_rom_delay_us
#elif __has_include("esp32/rom/ets_sys.h")
    #include "esp32/rom/ets_sys.h"
    #define delay_us ets_delay_us
#else
    #error "No suitable delay function found!"
#endif

// Define GPIO levels for the AM2301 sensor
#define AM2301_GPIO_LEVEL_HIGH 1
#define AM2301_GPIO_LEVEL_LOW  0

// Define and initialize the structure variable by calibration data
const am2301_signal_duration_t DURATION_START_SIGNAL_USEC = CALIBRATION_AM2301_START_SIGNAL_USEC;
const am2301_signal_duration_t DURATION_BUSMASTER_RELEASE_USEC = CALIBRATION_AM2301_BUSMASTER_RELEASE_USEC;
const am2301_signal_duration_t DURATION_STARTRESPONSE_LOW_USEC = CALIBRATION_AM2301_START_RESPONSE_LOW_USEC;
const am2301_signal_duration_t DURATION_STARTRESPONSE_HI_USEC = CALIBRATION_AM2301_START_RESPONSE_HI_USEC;
const am2301_signal_duration_t DURATION_SIGNAL_LOW_USEC = CALIBRATION_AM2301_SIGNAL_LOW_USEC;
const am2301_signal_duration_t DURATION_SIGNAL_0_USEC = CALIBRATION_AM2301_SIGNAL_0_USEC;
const am2301_signal_duration_t DURATION_SIGNAL_1_USEC = CALIBRATION_AM2301_SIGNAL_1_USEC;
const am2301_signal_duration_t DURATION_RELEASEBUS_USEC = CALIBRATION_AM2301_RELEASEBUS_USEC;

// GPIO interrupt handler
void IRAM_ATTR am2301_gpio_isr_handler(void* arg)
{
    am2301_context_t *context = (am2301_context_t*)arg;
    if (context == NULL) {
        return; // Exit if context is null
    }

    am2301_receiver_t *receiver = &context->_receiver;
    // Get the current signal level
    const int signal_level = gpio_get_level(context->gpio);

    if (signal_level == AM2301_GPIO_LEVEL_LOW && receiver->last_time_stored > 0) {
        // Skip processing the first low signal
        if (receiver->current_bit_index > 1 && // Skip the first 2 bits
            receiver->current_bit_index < TOTAL_AM2301_RESPONSE_BITS) {
                // Process the actual bit
                const size_t bit_received = receiver->current_bit_index - 2;
                // Calculate the duration of the high signal
                const int current_time = esp_timer_get_time();
                const int hi_sig_duration = current_time - receiver->last_time_stored;

                am2301_data_t *data = &context->data;
                // Determine if the received bit represents a '1'
                const uint8_t bit_value = hi_sig_duration >= DURATION_SIGNAL_1_USEC.MIN &&
                                          hi_sig_duration <= DURATION_SIGNAL_1_USEC.MAX;
                // Store the bit in the received data
                const size_t byte_position = bit_received / 8u;
                const size_t bit_position = 7 - (bit_received % 8u);
                data->bytes[byte_position] |= (bit_value << bit_position);
        }

        receiver->current_bit_index++;
        receiver->last_time_stored = 0;
        // Notify the main thread that all data has been received
        if (receiver->current_bit_index == TOTAL_AM2301_RESPONSE_BITS) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(receiver->semaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    } else {
        // Save the time of the current signal
        receiver->last_time_stored = esp_timer_get_time();
    }
}

// Internal functions
void clear_data(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return;
    }

    am2301_data_t *data = &context->data;
    data->fields.humidity_msb = 0u;
    data->fields.humidity_lsb = 0u;
    data->fields.temperature_msb = 0u;
    data->fields.temperature_lsb = 0u;
    data->fields.parity = 0u;

    am2301_receiver_t *receiver = &context->_receiver;
    receiver->current_bit_index = 0u;
    receiver->last_time_stored = 0;
}

void invalid_data(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return;
    }
    clear_data(context);
    am2301_data_t *data = &context->data;
    data->fields.parity = 0xFF;
}

void registered_interaptions(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return;
    }
    ESP_LOGD(TAG_AM2301_DHT21, "%s: GPIO pin %d", __FUNCTION__, context->gpio);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(context->gpio, am2301_gpio_isr_handler, (void*)context));
}

void unregistered_interaptions(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return;
    }
    ESP_LOGD(TAG_AM2301_DHT21, "%s: GPIO pin %d", __FUNCTION__, context->gpio);
    ESP_ERROR_CHECK(gpio_isr_handler_remove(context->gpio));
    gpio_uninstall_isr_service();
}

am2301_context_t* am2301_init(gpio_num_t pin)
{
    esp_err_t esp_err = ESP_OK;
    ESP_LOGI(TAG_AM2301_DHT21, "%s: Initializing sensor on GPIO pin %d", __FUNCTION__, pin);

    // Setup GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    esp_err|=gpio_config(&io_conf);
    esp_err|=gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    esp_err|=gpio_set_level(pin, AM2301_GPIO_LEVEL_HIGH);

    if (esp_err!=ESP_OK){
        ESP_LOGE(TAG_AM2301_DHT21, "%s Failed to GPIO initialization. \"%s\"", __FUNCTION__, esp_err_to_name(esp_err) );
        return NULL;
    }

    // Create context
    am2301_context_t* context = (am2301_context_t*)malloc(sizeof(am2301_context_t));
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s Failed to allocate memory for context", __FUNCTION__ );
        return NULL;
    }
    context->gpio = pin;
    // Reset buffered data
    invalid_data(context);

    // Create synchronization object
    context->_receiver.semaphore = xSemaphoreCreateBinary();
    if (context->_receiver.semaphore == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s Failed to create semaphore", __FUNCTION__ );
        am2301_free(context);
        return NULL;
    }

    return context;
}

bool am2301_free(am2301_context_t* context)
{
    if (context != NULL) {
        free(context);
        ESP_LOGI(TAG_AM2301_DHT21, "Freed AM2301 context resources");
        return true;
    }
    return false;
}

bool am2301_checksum_validate(am2301_data_t *data)
{
    const uint8_t sum = data->fields.humidity_msb +
                        data->fields.humidity_lsb +
                        data->fields.temperature_msb +
                        data->fields.temperature_lsb;
    ESP_LOGD(TAG_AM2301_DHT21, "%s: calculated parity == %.2X received value == %.2X", __FUNCTION__, sum, data->fields.parity);
    return sum == data->fields.parity;
}

bool am2301_acquire_data(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return false;
    }
    am2301_data_t *data = &context->data;

    ESP_LOGD(TAG_AM2301_DHT21, "Polling humidity and temperature data on GPIO pin %d", context->gpio);
    clear_data(context);
    registered_interaptions(context);

    // Start Signal: Before starting data transmission,
    // the microcontroller initiates communication by sending a low signal
    gpio_set_direction(context->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(context->gpio, AM2301_GPIO_LEVEL_LOW);
    delay_us(DURATION_START_SIGNAL_USEC.DEF);
    gpio_set_level(context->gpio, AM2301_GPIO_LEVEL_HIGH);
    delay_us(DURATION_BUSMASTER_RELEASE_USEC.DEF);
    gpio_set_direction(context->gpio, GPIO_MODE_INPUT);

    // wait to receive 40 bits from sensor
    bool bres = true;
    const uint32_t timeout_value_msec = (( TOTAL_AM2301_RESPONSE_BITS *
                             ( DURATION_SIGNAL_LOW_USEC.MAX + DURATION_SIGNAL_1_USEC.MAX ) +
                             DURATION_START_SIGNAL_USEC.MAX + DURATION_BUSMASTER_RELEASE_USEC.MAX)
                             + 999 ) / 1000;
    if (xSemaphoreTake(context->_receiver.semaphore, pdMS_TO_TICKS(timeout_value_msec)) != pdTRUE) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s: Timeout [%u msec] occurred while receiving data from the sensor", __FUNCTION__, timeout_value_msec);
        invalid_data(context);
        bres = false;
    }

    // Reset output signal to high
    gpio_set_direction(context->gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(context->gpio, AM2301_GPIO_LEVEL_HIGH);

    if ( bres && am2301_checksum_validate(data) == false ){
        ESP_LOGE(TAG_AM2301_DHT21, "%s: Checksum mismatch. pin %d. data [%.2X %.2X %.2X %.2X %.2X]", __FUNCTION__,
                context->gpio, context->data.bytes[0], context->data.bytes[1], context->data.bytes[2],
                context->data.bytes[3], context->data.bytes[4] );
        invalid_data(context);
        bres = false;
    }

    unregistered_interaptions(context);
    return bres;
}

float am2301_temperature(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return 0.0;
    }
    am2301_data_t *data = &context->data;
    float temperature = data->fields.temperature_msb & 0x7F;
    temperature *= 256.0;
    temperature += data->fields.temperature_lsb;
    temperature /= 10.0;
    if (data->fields.temperature_msb & 0x80)
        temperature *= -1;

    ESP_LOGD(TAG_AM2301_DHT21, "get temperature =%.2f", temperature);
    return temperature;
}

float am2301_humidity(am2301_context_t *context)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return 0.0;
    }
    am2301_data_t *data = &context->data;
    float humidity = data->fields.humidity_msb;
    humidity *= 256.0;
    humidity += data->fields.humidity_lsb;
    humidity /= 10.0;

    ESP_LOGD(TAG_AM2301_DHT21, "get humidity =%.2f", humidity);
    return humidity;
}

// public API implementation
bool am2301_poll_temperature(am2301_context_t *context, float *temperature)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return false;
    }
    ESP_LOGI(TAG_AM2301_DHT21, "%s: GPIO pin %d", __FUNCTION__, context->gpio);
    if (am2301_acquire_data(context) ){
        *temperature = am2301_temperature(context);
        ESP_LOGD(TAG_AM2301_DHT21, "%s: GPIO pin %d temperature: %.2f°С", __FUNCTION__, context->gpio, *temperature);
        return true;
    }
    return false;
}

bool am2301_poll_humidity(am2301_context_t *context, float *humidity)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return false;
    }
    ESP_LOGI(TAG_AM2301_DHT21, "%s: GPIO pin %d", __FUNCTION__, context->gpio);
    if (am2301_acquire_data(context) ){
        *humidity = am2301_humidity(context);
        ESP_LOGD(TAG_AM2301_DHT21, "%s: GPIO pin %d humidity: %.2f%%", __FUNCTION__, context->gpio, *humidity);
        return true;
    }
    return false;
}

bool am2301_poll_data(am2301_context_t *context, float *temperature, float *humidity)
{
    if (context == NULL) {
        ESP_LOGE(TAG_AM2301_DHT21, "%s context is NULL", __FUNCTION__);
        return false;
    }
    ESP_LOGI(TAG_AM2301_DHT21, "%s: GPIO pin %d", __FUNCTION__, context->gpio);
    if (am2301_acquire_data(context) ){
        *temperature = am2301_temperature(context);
        *humidity = am2301_humidity(context);
        ESP_LOGD(TAG_AM2301_DHT21, "%s: GPIO pin %d temperature: %.2f°С, humidity: %.2f%%", __FUNCTION__, context->gpio, *temperature, *humidity);
        return true;
    }
    return false;
}