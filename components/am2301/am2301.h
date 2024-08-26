/*
 * AM2301-DHT21 Sensor Interface Component
 *
 * This component provides functionality for interfacing with the AM2301-DHT21 sensor.
 * It includes functions to initialize the sensor, read temperature and humidity data,
 * and handle any errors that may occur during sensor communication.
 *
 * Dependencies:
 * - ESP-IDF (Espressif IoT Development Framework)
 * - Necessary drivers and libraries for GPIO communication
 *
 * License:
 * This component is released under the MIT License. See the LICENSE file for details.
 * You are free to use, modify, and distribute this code as long as the above license is included in all copies or substantial portions of the code.
 *
 * Disclaimer:
 * This component is provided "as is", without warranty of any kind. The authors are not liable for any damages arising from the use of this component.
 * Use it at your own risk.
 *
 * Author: Andrii Solomai
 * Date: Aug 2024
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/*
 * Calibration data for the AM2301-DHT21 sensor.
 */

// Structure to setup timing calibration
typedef struct {
    uint32_t MIN; // Minimum value in microseconds
    uint32_t MAX; // Maximum value in microseconds
    uint32_t DEF; // Default value in microseconds
} am2301_signal_duration_t;

// Host the start signal down time
// Minimum value:    800 microseconds
// Maximum value:  20000 microseconds
// Default value:  10000 microseconds
#define CALIBRATION_AM2301_START_SIGNAL_USEC { 800, 20000, 10000 }

// Duration for which the bus master releases the bus after the start signal
// Minimum value:     20 microseconds
// Maximum value:    200 microseconds
// Default value:     30 microseconds
#define CALIBRATION_AM2301_BUSMASTER_RELEASE_USEC { 20, 200, 30 }

// Sensor data bus (SDA) is pulled down response to host the start signal.
// Minimum value:     75 microseconds
// Maximum value:     85 microseconds
// Default value:     80 microseconds
#define CALIBRATION_AM2301_START_RESPONSE_LOW_USEC { 75, 85, 80 }

// Sensor data bus (SDA) is pulled up  response to host the start signal.
// Minimum value:     75 microseconds
// Maximum value:     85 microseconds
// Default value:     80 microseconds
#define CALIBRATION_AM2301_START_RESPONSE_HI_USEC { 75, 85, 80 }

// Signal low time separation between data bits
// Minimum value:     45 microseconds
// Maximum value:     55 microseconds
// Default value:     50 microseconds
#define CALIBRATION_AM2301_SIGNAL_LOW_USEC { 45, 55, 50 }

// High signal duration indicating a data bit of 0.
// Minimum value:     20 microseconds
// Maximum value:     30 microseconds
// Default value:     26 microseconds
#define CALIBRATION_AM2301_SIGNAL_0_USEC { 20, 30, 26 }

// High signal duration indicating a data bit of 1.
// Minimum value:     40 microseconds
// Maximum value:     80 microseconds
// Default value:     70 microseconds
#define CALIBRATION_AM2301_SIGNAL_1_USEC { 40, 80, 70 }

// Sensor to release the bus time
// Minimum value:     45 microseconds
// Maximum value:     55 microseconds
// Default value:     50 microseconds
#define CALIBRATION_AM2301_RELEASEBUS_USEC { 45, 55, 50 }

/*
 * Data types
 */

#define TAG_AM2301_DHT21 "AM2301_DHT21_SENSOR"

// Union to access AM2301-DHT21 data as bytes or individual fields
typedef union {
    uint8_t bytes[5]; // Raw data from the sensor
    struct {
        uint8_t humidity_msb;    // Hi byte of humidity value
        uint8_t humidity_lsb;    // Low byte of humidity value
        uint8_t temperature_msb; // Hi byte of temperature value
        uint8_t temperature_lsb; // Low byte of temperature value
        uint8_t parity;          // Parity byte
    } fields;
} am2301_data_t;

// Total number of bits in the sensor's response:
// 2 bits for the start signal and 40 bits for data
#define TOTAL_AM2301_RESPONSE_BITS 42
// Structure to store decoded signal timings from the AM2301 sensor
// It captures the duration of each signal bit for decoding the sensor's response.
// For details, refer to the sensor's datasheet.
typedef struct {
    // Index of the current bit in the sequence
    size_t current_bit_index;
    // Time of the last signal change
    int last_time_stored;
    // Synchronization object to coordinate between interrupt handler and main code
    SemaphoreHandle_t semaphore;
} am2301_receiver_t;

// Sensor context to store all data used for data retrieval
typedef struct {
    // GPIO pin used for the sensor
    gpio_num_t gpio;
    // Binary data received from the sensor
    am2301_data_t data;
    // Internal buffer to decode bits by timing
    am2301_receiver_t _receiver;
} am2301_context_t;

/*
 * Public APIs
 */

/**
 * @brief Initializes the AM2301-DHT21 temperature and humidity sensor GPIO pin.
 *
 * Configures the GPIO pin for communication with the AM2301-DHT21 sensor and initializes
 * the necessary data structures for sensor communication.
 *
 * @param pin GPIO number to which the AM2301-DHT21 sensor is connected.
 * @return Pointer to the context structure for the sensor, or NULL if initialization fails.
 */
am2301_context_t* am2301_init(gpio_num_t pin);

/**
 * @brief Frees the resources allocated for the AM2301-DHT21 sensor.
 *
 * Releases any memory and resources associated with the sensor context.
 *
 * @param context Pointer to the sensor context to be freed.
 * @return True if the resources were successfully freed, false otherwise.
 */
bool am2301_free(am2301_context_t* context);

/**
 * @brief Polls the AM2301-DHT21 sensor and retrieves the temperature reading.
 *
 * Communicates with the sensor to acquire the current temperature data.
 *
 * @param context Pointer to the sensor context.
 * @param temperature Pointer to a float where the temperature value will be stored.
 * @return True if the temperature was successfully retrieved, false otherwise.
 */
bool am2301_poll_temperature(am2301_context_t *context, float *temperature);

/**
 * @brief Polls the AM2301-DHT21 sensor and retrieves the humidity reading.
 *
 * Communicates with the sensor to acquire the current humidity data.
 *
 * @param context Pointer to the sensor context.
 * @param humidity Pointer to a float where the humidity value will be stored.
 * @return True if the humidity was successfully retrieved, false otherwise.
 */
bool am2301_poll_humidity(am2301_context_t *context, float *humidity);

/**
 * @brief Polls the AM2301-DHT21 sensor and retrieves both temperature and humidity readings.
 *
 * Communicates with the sensor to acquire the current temperature and humidity data.
 *
 * @param context Pointer to the sensor context.
 * @param temperature Pointer to a float where the temperature value will be stored.
 * @param humidity Pointer to a float where the humidity value will be stored.
 * @return True if both temperature and humidity were successfully retrieved, false otherwise.
 */
bool am2301_poll_data(am2301_context_t *context, float *temperature, float *humidity);
