/**
 * @file sensor_t_a_h.c
 * @brief Implementation of Temperature, Air pressure, and Humidity sensor interface for ESP32
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensor_t_a_h.h"
#include "sensor_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

/* Define tag for ESP logging */
static const char *TAG = "SENSOR_T_A_H";

/* Constants */
#define STACK_SIZE 4096       // Stack size for the polling task
#define TASK_PRIORITY 5       // Priority of the polling task

/* Task handle */
static TaskHandle_t polling_task_handle = NULL;

/* Context structure */
typedef struct {
    struct bme280_dev device; // BME280 device structure
    sensor_state_t state;     // Current state of the sensor
    bool task_running;        // Flag to control task execution
    uint32_t poll_interval_ms; // Polling interval in milliseconds
    uint32_t timeout_ms;      // Timeout for operations

    // Circular buffer for data storage
    t_a_h_data_t buffer[SENSOR_BUFFER_SIZE]; // Buffer for data storage
    uint8_t head;                            // Buffer head index
    uint8_t tail;                            // Buffer tail index
    uint8_t count;                           // Number of samples in buffer
    uint8_t retry_count;                     // Number of retries
    uint8_t error_count;                     // Number of errors
    SemaphoreHandle_t mutex;                 // FreeRTOS mutex for thread-safe access
} t_a_h_context_t;

/* Static variables */
static t_a_h_context_t *sensor_ctx = NULL;

/* Forward declarations for static functions */
static void polling_task_func(void *arg);
static void buffer_push(t_a_h_data_t *data);
static void buffer_get_average(t_a_h_data_t *result);
static uint64_t get_timestamp_ms(void);

/**
 * @brief Push data to the circular buffer
 */
static void buffer_push(t_a_h_data_t *data) {
    if (sensor_ctx->count < SENSOR_BUFFER_SIZE) {
        sensor_ctx->count++;
    } else {
        // Remove oldest data
        sensor_ctx->tail = (sensor_ctx->tail + 1) % SENSOR_BUFFER_SIZE;
    }
    sensor_ctx->buffer[sensor_ctx->head] = *data;
    sensor_ctx->head = (sensor_ctx->head + 1) % SENSOR_BUFFER_SIZE;
}

/**
 * @brief Get average data from the buffer
 */
static void buffer_get_average(t_a_h_data_t *result) {
    if (sensor_ctx->count == 0) {
        result->temperature = 0;
        result->humidity = 0;
        result->pressure = 0;
        result->timestamp = 0;
        return;
    }

    float temp_sum = 0;
    float hum_sum = 0;
    float press_sum = 0;
    uint32_t latest_timestamp = 0;

    uint8_t count = sensor_ctx->count;
    uint8_t idx = sensor_ctx->tail;
    
    for (uint8_t i = 0; i < count; i++) {
        temp_sum += sensor_ctx->buffer[idx].temperature;
        hum_sum += sensor_ctx->buffer[idx].humidity;
        press_sum += sensor_ctx->buffer[idx].pressure;
        if (sensor_ctx->buffer[idx].timestamp > latest_timestamp) {
            latest_timestamp = sensor_ctx->buffer[idx].timestamp;
        }
        idx = (idx + 1) % SENSOR_BUFFER_SIZE;
    }

    result->temperature = temp_sum / count;
    result->humidity = hum_sum / count;
    result->pressure = press_sum / count;
    result->timestamp = latest_timestamp;
}

/**
 * @brief Initialize the sensor and create the context
 */
esp_err_t t_a_h_sensor_init(uint32_t poll_interval_ms, uint32_t timeout_ms)
{
    int8_t rslt;

    /* Allocate and initialize the sensor context */
    sensor_ctx = (t_a_h_context_t *)calloc(1, sizeof(t_a_h_context_t));
    if (sensor_ctx == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate context");
        return ESP_ERR_NO_MEM;
    }

    /* Store polling interval and timeout */
    sensor_ctx->poll_interval_ms = poll_interval_ms;
    sensor_ctx->timeout_ms = timeout_ms;

    /* Initialize mutex */
    sensor_ctx->mutex = xSemaphoreCreateMutex();
    if (sensor_ctx->mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(sensor_ctx);
        sensor_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize buffer */
    sensor_ctx->head = 0;
    sensor_ctx->tail = 0;
    sensor_ctx->count = 0;
    sensor_ctx->retry_count = 0;
    sensor_ctx->error_count = 0;

    /* Initialize the sensor using the API */
    rslt = sensor_api_init(&sensor_ctx->device);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        vSemaphoreDelete(sensor_ctx->mutex);
        free(sensor_ctx);
        sensor_ctx = NULL;
        return ESP_FAIL;
    }

    /* Configure sensor settings */
    struct bme280_settings settings;
    rslt = sensor_api_get_settings(&sensor_ctx->device, &settings);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to get sensor settings");
        vSemaphoreDelete(sensor_ctx->mutex);
        free(sensor_ctx);
        sensor_ctx = NULL;
        return ESP_FAIL;
    }

    /* Configure for optimal performance for our use case */
    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = sensor_api_configure(&sensor_ctx->device, &settings);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to configure sensor settings");
        vSemaphoreDelete(sensor_ctx->mutex);
        free(sensor_ctx);
        sensor_ctx = NULL;
        return ESP_FAIL;
    }

    /* Set sensor to normal power mode */
    rslt = sensor_api_set_power_mode(&sensor_ctx->device, BME280_POWERMODE_NORMAL);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to set sensor mode");
        vSemaphoreDelete(sensor_ctx->mutex);
        free(sensor_ctx);
        sensor_ctx = NULL;
        return ESP_FAIL;
    }

    /* Update state */
    sensor_ctx->state = SENSOR_STATE_INITIALIZED;
    sensor_ctx->task_running = false;

    ESP_LOGI(TAG, "Sensor initialized successfully with %d ms polling interval", poll_interval_ms);
    return ESP_OK;
}

/**
 * @brief Start the polling task
 */
esp_err_t t_a_h_start_polling(void)
{
    /* Check if sensor is initialized */
    if (sensor_ctx == NULL || sensor_ctx->state != SENSOR_STATE_INITIALIZED)
    {
        ESP_LOGE(TAG, "Cannot start polling - invalid state");
        return ESP_ERR_INVALID_STATE;
    }

    /* Take mutex to update state */
    if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(sensor_ctx->timeout_ms)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Set task running flag */
    sensor_ctx->task_running = true;


    /* Create polling task */
    BaseType_t ret = xTaskCreate(
        polling_task_func,
        "bme280_poll",
        STACK_SIZE,
        sensor_ctx,
        TASK_PRIORITY,
        &polling_task_handle);

    if (ret != pdPASS)
    {
        sensor_ctx->task_running = false;
        xSemaphoreGive(sensor_ctx->mutex);
        ESP_LOGE(TAG, "Failed to create polling task");
        return ESP_ERR_NO_MEM;
    }

    /* Update state */
    sensor_ctx->state = SENSOR_STATE_POLLING;

    /* Release mutex */
    xSemaphoreGive(sensor_ctx->mutex);

    ESP_LOGI(TAG, "Polling started with %d ms interval", sensor_ctx->poll_interval_ms);
    return ESP_OK;
}

/**
 * @brief Stop the polling task
 */
esp_err_t t_a_h_stop_polling(void)
{
    esp_err_t ret = ESP_OK;
    
    /* Check if sensor is initialized */
    if (sensor_ctx == NULL)
    {
        ESP_LOGE(TAG, "Cannot stop polling - context is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    
    /* Take mutex to update state */
    if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(sensor_ctx->timeout_ms)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    /* Set task running flag to false to signal task to exit */
    sensor_ctx->task_running = false;
    
    /* Release mutex */
    xSemaphoreGive(sensor_ctx->mutex);
    
    /* Wait for task to complete exit */
    if (polling_task_handle != NULL)
    {
        polling_task_handle = NULL;
    }
    
    /* Update state */
    if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(sensor_ctx->timeout_ms)) == pdTRUE)
    {
        sensor_ctx->state = SENSOR_STATE_INITIALIZED;
        xSemaphoreGive(sensor_ctx->mutex);
    }
    
    
    ESP_LOGI(TAG, "Polling stopped");
    return ret;
}

/**
 * @brief Get the current state of the sensor
 */
sensor_state_t t_a_h_get_state(void)
{
    sensor_state_t state = SENSOR_STATE_UNINITIALIZED;

    if (sensor_ctx != NULL)
    {
        if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(sensor_ctx->timeout_ms)) == pdTRUE)
        {
            state = sensor_ctx->state;
            xSemaphoreGive(sensor_ctx->mutex);
        }
    }

    return state;
}

/**
 * @brief Get the latest sensor data
 */
esp_err_t t_a_h_get_data(t_a_h_data_t *data, uint32_t timeout_ms)
{
    if (sensor_ctx == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (sensor_ctx->count == 0) {
        xSemaphoreGive(sensor_ctx->mutex);
        return ESP_ERR_NOT_FOUND;
    }

    // Get latest data
    uint8_t latest_idx = (sensor_ctx->head - 1 + SENSOR_BUFFER_SIZE) % SENSOR_BUFFER_SIZE;
    *data = sensor_ctx->buffer[latest_idx];

    xSemaphoreGive(sensor_ctx->mutex);
    return ESP_OK;
}

/**
 * @brief Get the average data from the buffer
 */
esp_err_t t_a_h_get_average_data(t_a_h_data_t *data)
{
    if (sensor_ctx == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sensor_ctx->mutex, pdMS_TO_TICKS(sensor_ctx->timeout_ms)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    buffer_get_average(data);

    xSemaphoreGive(sensor_ctx->mutex);
    return ESP_OK;
}

/**
 * @brief Deinitialize the sensor
 */
esp_err_t t_a_h_deinit(void)
{
    esp_err_t ret = ESP_OK;
    
    /* Stop polling if running */
    if (sensor_ctx != NULL && sensor_ctx->state == SENSOR_STATE_POLLING)
    {
        ret = t_a_h_stop_polling();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to stop polling");
            return ret;
        }
    }
    
    /* Clean up resources */
    if (sensor_ctx != NULL)
    {
        /* Set sensor to sleep mode */
        sensor_api_set_power_mode(&sensor_ctx->device, BME280_POWERMODE_SLEEP);
        
        /* Delete mutex if it exists */
        if (sensor_ctx->mutex != NULL)
        {
            vSemaphoreDelete(sensor_ctx->mutex);
            sensor_ctx->mutex = NULL;
        }
        
        /* Free context */
        free(sensor_ctx);
        sensor_ctx = NULL;
    }
    
    
    ESP_LOGI(TAG, "Sensor deinitialized");
    return ret;
}

/**
 * @brief Polling task function
 */
static void polling_task_func(void *arg)
{
    t_a_h_context_t *ctx = (t_a_h_context_t *)arg;
    struct bme280_data comp_data;
    t_a_h_data_t sensor_data;
    int8_t rslt;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    
    ESP_LOGI(TAG, "Polling task started");


    while (ctx->task_running) {
        
        /* Attempt to read data from the sensor */
        rslt = sensor_api_read_data(&ctx->device, &comp_data);
        if (rslt != BME280_OK) {
            ctx->error_count++;
            ESP_LOGW(TAG, "Failed to read sensor data, error count: %d", ctx->error_count);
            
            /* Try to recover after multiple errors */
            if (ctx->error_count >= 5) {
                ESP_LOGE(TAG, "Too many sensor errors, attempting recovery");
                ctx->retry_count++;
                ctx->error_count = 0;
                
                /* Try to recover by reinitializing sensor */
                if (ctx->retry_count >= 3) {
                    ESP_LOGE(TAG, "Recovery attempts exhausted, sensor failure");
                    ctx->state = SENSOR_STATE_ERROR;
                    break;
                }
                
                /* Attempt to reinitialize */
                struct bme280_settings settings;
                sensor_api_get_settings(&ctx->device, &settings);
                sensor_api_set_power_mode(&ctx->device, BME280_POWERMODE_NORMAL);
                sensor_api_configure(&ctx->device, &settings);
                
            }
            
            /* Wait before trying again */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        /* Reset error counts if successful */
        ctx->error_count = 0;
        ctx->retry_count = 0;
        
        /* Convert to more usable units if needed */
        sensor_data.temperature = comp_data.temperature;
        sensor_data.pressure = comp_data.pressure / 100.0f; // Convert Pa to hPa
        sensor_data.humidity = comp_data.humidity;
        sensor_data.timestamp = get_timestamp_ms();
        
        /* Try to take mutex to update data */
        if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Add to buffer */
            buffer_push(&sensor_data);
            /* Update state if needed */
            if (ctx->state != SENSOR_STATE_POLLING) {
                ctx->state = SENSOR_STATE_POLLING;
            }
            xSemaphoreGive(ctx->mutex);
        }
        
        
        /* Log data occasionally */
        static uint32_t log_counter = 0;
        log_counter++;
        if (log_counter % 10 == 0) { // Log every 10th reading
            ESP_LOGI(TAG, "Sensor data: Temp=%.2f°C, Pressure=%.2fhPa, Humidity=%.2f%%",
                   sensor_data.temperature, sensor_data.pressure, sensor_data.humidity);
        }
        
        /* Delay until next measurement, using the calculated interval */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ctx->poll_interval_ms));
        
    }
    
    /* Set sensor to sleep mode to save power */
    sensor_api_set_power_mode(&ctx->device, BME280_POWERMODE_SLEEP);
    
    /* Task cleanup */
    ESP_LOGI(TAG, "Polling task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Get current timestamp in milliseconds
 */
static uint64_t get_timestamp_ms(void)
{
    return esp_log_timestamp();
}