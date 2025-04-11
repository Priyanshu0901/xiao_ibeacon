#include "power_manager.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

static const char *TAG = "power_manager";

esp_err_t power_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing power manager");
    
    // Configure wakeup sources
    esp_sleep_enable_timer_wakeup(1);  // Will be reconfigured before sleep
    
    // Print wakeup reason
    ESP_LOGI(TAG, "Wakeup reason: %s", power_manager_get_wakeup_reason());
    
    return ESP_OK;
}

esp_err_t power_manager_enter_deep_sleep(uint32_t sleep_time_ms)
{
    ESP_LOGI(TAG, "Entering deep sleep for %lu ms", sleep_time_ms);
    
    // Configure wakeup timer
    esp_sleep_enable_timer_wakeup(sleep_time_ms * 1000);  // Convert to microseconds
    
    // Enter deep sleep
    esp_deep_sleep_start();
    
    // This line will never be reached
    return ESP_OK;
}

const char* power_manager_get_wakeup_reason(void)
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            return "External signal using RTC_IO";
        case ESP_SLEEP_WAKEUP_EXT1:
            return "External signal using RTC_CNTL";
        case ESP_SLEEP_WAKEUP_TIMER:
            return "Timer";
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            return "Touchpad";
        case ESP_SLEEP_WAKEUP_ULP:
            return "ULP program";
        case ESP_SLEEP_WAKEUP_GPIO:
            return "GPIO";
        case ESP_SLEEP_WAKEUP_UART:
            return "UART";
        default:
            return "Unknown";
    }
} 