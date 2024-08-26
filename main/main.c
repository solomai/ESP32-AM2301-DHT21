/*
 * AM2301-DHT21 Sensor Example
 *
 * This example code is released into the Public Domain (or is licensed under CC0, at your option).
 *
 * Unless required by applicable law or agreed upon in writing,
 * this software is provided "AS IS", without any warranties or conditions
 * of any kind, either express or implied.
 */

// include freertos lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
// include components
#include "am2301.h"

#define TAG "EXAMPLE_FOR_AM2301_DHT21_SENSOR"

// Define the GPIO pin number for the LED used for blinking
#define BLINK_GPIO         GPIO_NUM_2

// Define the GPIO pin number where the AM2301-DHT21 sensor is connected
#define AM2301_SENSOR_GPIO GPIO_NUM_4

// Define the GPIO level for turning the LED on
#define LED_ON  1

// Define the GPIO level for turning the LED off
#define LED_OFF 0

// Duration in milliseconds for LED blink
#define LED_BLINK_DURATION 1000

// Interval in milliseconds for LED blink
#define LED_BLINK_INTERVAL 500

// Interval in milliseconds to read data from sensor
#define SENSOR_READ_INTERVAL 10000

void app_main(void)
{
    // Init blink LED
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize the AM2301-DHT21 sensor.
    // Remember to call function am2301_free to release the context and free resources after use.
    am2301_context_t *am2301_context = am2301_init(AM2301_SENSOR_GPIO);
    if (am2301_context == NULL) {
        ESP_LOGE(TAG, "%s: Cannot initialize sensor AM2301-DHT21.", __FUNCTION__);
        return; // Exit if initialization failed
    }

    // Infinite loop to continuously fetch humidity and temperature data from the AM2301 sensor
    while (1) {
        gpio_set_level(BLINK_GPIO, LED_ON);
        float temperature = 0.0;
        float humidity = 0.0;

        if (am2301_poll_data(am2301_context, &temperature, &humidity)) {
            printf("AM2301-DHT21 return values: temperature = %.2f°C, humidity = %.2f%%\n", temperature, humidity);
        } else {
            printf("Failed to read data from AM2301-DHT21 sensor.\n");
            gpio_set_level(BLINK_GPIO, LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL));
            for (size_t cnt = 0; cnt < 3; cnt++) {
                gpio_set_level(BLINK_GPIO, LED_ON);
                vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION));
                gpio_set_level(BLINK_GPIO, LED_OFF);
                vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL));
            }
        }
        gpio_set_level(BLINK_GPIO, LED_OFF);

        // Wait for the next iteration
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
    }

    // Free the AM2301 context if the loop ever exits
    am2301_free(am2301_context);
}
