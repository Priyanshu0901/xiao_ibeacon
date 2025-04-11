/**
 * @file sensor_t_a_h.h
 * @brief Temperature, Air pressure, and Humidity sensor interface for ESP32
 *
 * This module provides a thread-safe interface for reading temperature,
 * air pressure, and humidity data from a BME280 sensor with filtering.
 */

#ifndef SENSOR_T_A_H_H
#define SENSOR_T_A_H_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* Error codes */
#define T_A_H_E_NULL_PTR INT8_C(-1)
#define T_A_H_E_INVALID_STATE INT8_C(-2)
#define T_A_H_E_TASK_CREATE INT8_C(-3)
#define T_A_H_E_QUEUE_CREATE INT8_C(-4)
#define T_A_H_E_QUEUE_EMPTY INT8_C(-5)

/**
 * @brief Sensor state enumeration
 */
typedef enum {
    SENSOR_STATE_UNINITIALIZED = 0,
    SENSOR_STATE_INITIALIZED,
    SENSOR_STATE_POLLING,
    SENSOR_STATE_OK,
    SENSOR_STATE_FAILING,
    SENSOR_STATE_ERROR
} sensor_state_t;

/**
 * @brief Buffer configuration
 */
#define SENSOR_BUFFER_SIZE 10  // Number of samples to keep
#define MAX_RETRY_COUNT 3      // Maximum number of retries before marking as failing
#define MAX_ERROR_COUNT 5      // Maximum number of errors before marking as error

/**
 * @brief Structure to hold sensor data
 */
typedef struct
{
    float temperature;  // Temperature in Celsius
    float humidity;     // Humidity in percentage
    float pressure;     // Pressure in hPa
    uint32_t timestamp; // Timestamp in milliseconds
} t_a_h_data_t;

/**
 * @brief Buffer structure
 */
typedef struct {
    t_a_h_data_t data[SENSOR_BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t retry_count;
    uint8_t error_count;
    sensor_state_t state;
} sensor_buffer_t;

/**
 * @brief Initialize the sensor
 * 
 * @param poll_interval_ms Polling interval in milliseconds
 * @param timeout_ms Timeout for sensor operations in milliseconds
 * @return ESP_OK on success
 */
esp_err_t t_a_h_sensor_init(uint32_t poll_interval_ms, uint32_t timeout_ms);

/**
 * @brief Start the sensor polling task
 * 
 * @return ESP_OK on success
 */
esp_err_t t_a_h_start_polling(void);

/**
 * @brief Stop the sensor polling task
 * 
 * @return ESP_OK on success
 */
esp_err_t t_a_h_stop_polling(void);

/**
 * @brief Get the latest sensor data
 * 
 * @param data Pointer to store the sensor data
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t t_a_h_get_data(t_a_h_data_t *data, uint32_t timeout_ms);

/**
 * @brief Get the average sensor data from buffer
 * 
 * @param data Pointer to store the averaged sensor data
 * @return ESP_OK on success
 */
esp_err_t t_a_h_get_average_data(t_a_h_data_t *data);

/**
 * @brief Get the current sensor state
 * 
 * @return Current sensor state
 */
sensor_state_t t_a_h_get_state(void);

/**
 * @brief Cleanup sensor resources
 * 
 * @return ESP_OK on success
 */
esp_err_t t_a_h_deinit(void);

#endif /* SENSOR_T_A_H_H */