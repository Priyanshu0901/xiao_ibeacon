#ifndef SENSOR_API_H
#define SENSOR_API_H

#include "sensor_t_a_h.h"
#include "bme280.h"
#include "common.h"

/**
 * @brief Initialize the BME280 sensor hardware
 * 
 * @param dev Pointer to BME280 device structure
 * @return int8_t BME280_OK on success, error code otherwise
 */
int8_t sensor_api_init(struct bme280_dev *dev);

/**
 * @brief Read raw sensor data from BME280
 * 
 * @param dev Pointer to BME280 device structure
 * @param data Pointer to store the raw sensor data
 * @return int8_t BME280_OK on success, error code otherwise
 */
int8_t sensor_api_read_data(struct bme280_dev *dev, struct bme280_data *data);

/**
 * @brief Configure BME280 sensor settings
 * 
 * @param dev Pointer to BME280 device structure
 * @param settings Pointer to settings structure
 * @return int8_t BME280_OK on success, error code otherwise
 */
int8_t sensor_api_configure(struct bme280_dev *dev, struct bme280_settings *settings);

/**
 * @brief Get current sensor settings
 * 
 * @param dev Pointer to BME280 device structure
 * @param settings Pointer to store current settings
 * @return int8_t BME280_OK on success, error code otherwise
 */
int8_t sensor_api_get_settings(struct bme280_dev *dev, struct bme280_settings *settings);

/**
 * @brief Set sensor power mode
 * 
 * @param dev Pointer to BME280 device structure
 * @param mode Power mode to set
 * @return int8_t BME280_OK on success, error code otherwise
 */
int8_t sensor_api_set_power_mode(struct bme280_dev *dev, uint8_t mode);

#endif // SENSOR_API_H 