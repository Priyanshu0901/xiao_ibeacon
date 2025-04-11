#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the power manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_manager_init(void);

/**
 * @brief Enter deep sleep mode for a specified duration
 * @param sleep_time_ms Duration of deep sleep in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_manager_enter_deep_sleep(uint32_t sleep_time_ms);

/**
 * @brief Get the wakeup reason
 * @return Wakeup reason as a string
 */
const char* power_manager_get_wakeup_reason(void);

#endif // POWER_MANAGER_H 