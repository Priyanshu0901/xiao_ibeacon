#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sensor_t_a_h.h"
#include "ble_beacon.h"
#include "power_manager.h"

// Configuration

#define ADV_TIME_MS      500   // 1 seconds advertising time (increased for better discovery)
#define POLL_INTERVAL_MS 150   // 0.15 second polling interval
#define TIMEOUT_MS       2000   // 5 seconds timeout
#define SLEEP_TIME_MS    30000 - ADV_TIME_MS - POLL_INTERVAL_MS// 30 seconds sleep time

static const char* TAG = "main";

// Forward declarations
static void safe_cleanup(void);

void app_main(void)
{
    // Initialize NVS - required for controller to store calibration data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %d", ret);
        return;
    }

    // Initialize power manager first
    ESP_LOGI(TAG, "Initializing power manager component");
    ret = power_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power manager: %d", ret);
        safe_cleanup();
        return;
    }

    // Initialize sensor component
    ESP_LOGI(TAG, "Initializing sensor component");
    ret = t_a_h_sensor_init(POLL_INTERVAL_MS, TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor: %d", ret);
        safe_cleanup();
        return;
    }

    // Start sensor polling
    ESP_LOGI(TAG, "Starting sensor polling");
    ret = t_a_h_start_polling();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start sensor polling: %d", ret);
        safe_cleanup();
        return;
    }

    // Wait for first data
    ESP_LOGI(TAG, "Waiting for sensor data");
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS * 2));

    // Get sensor data
    t_a_h_data_t sensor_data;
    ret = t_a_h_get_data(&sensor_data, TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get sensor data: %d", ret);
        safe_cleanup();
        return;
    }

    // Initialize BLE component after we have data (to avoid wasting power if sensor fails)
    ESP_LOGI(TAG, "Initializing BLE beacon component");
    ret = ble_beacon_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE beacon: %d", ret);
        safe_cleanup();
        return;
    }

    // Advertise sensor data
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Starting BLE temperature sensor advertisement");
    ESP_LOGI(TAG, "Device name: ESP_TempSensor");
    ESP_LOGI(TAG, "Sensor readings:");
    ESP_LOGI(TAG, "  - Temperature: %.2f°C", sensor_data.temperature);
    ESP_LOGI(TAG, "  - Humidity: %.0f%%", sensor_data.humidity);
    ESP_LOGI(TAG, "  - Pressure: %.1f hPa", sensor_data.pressure);
    ESP_LOGI(TAG, "Advertising for %d ms", ADV_TIME_MS);
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "");
    
    ret = ble_beacon_start(&sensor_data, ADV_TIME_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE advertising may have failed: %d", ret);
        // Continue anyway
    } else {
        ESP_LOGI(TAG, "BLE advertising completed successfully");
    }

    // Cleanup in a safe manner
    safe_cleanup();
    
    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep for %u ms", SLEEP_TIME_MS);
    ESP_ERROR_CHECK(power_manager_enter_deep_sleep(SLEEP_TIME_MS));
}

// Cleanup resources safely
static void safe_cleanup(void)
{
    esp_err_t ret;
    
    // Stop BLE beacon first
    ESP_LOGI(TAG, "Stopping BLE advertising");
    ble_beacon_stop();
    
    // Allow time for BLE to stop properly
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Deinitialize BLE immediately to free up resources
    ESP_LOGI(TAG, "Deinitializing BLE beacon");
    ble_beacon_deinit();
    
    // Wait for BLE to fully deinitialize
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Stopping sensor polling");
    ret = t_a_h_stop_polling();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not stop sensor polling gracefully: %d", ret);
    }
    
    // Wait longer for the polling task to terminate itself - avoid forced deletion
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Call deinit but be ready to handle errors
    ESP_LOGI(TAG, "Deinitializing sensor");
    ret = t_a_h_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Sensor deinitialization reported an error: %d", ret);
        // We can't do much about it at this point, just proceed to sleep
    }
    
    // Final delay before sleep
    vTaskDelay(pdMS_TO_TICKS(100));
}
