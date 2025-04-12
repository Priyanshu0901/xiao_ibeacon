#ifndef SENSOR_API_H
#define SENSOR_API_H

#include "sensor_t_a_h.h"
#include "bme68x.h"
#include "common.h"

/**
 * @brief Initialize the BME68X sensor hardware
 * 
 * @param dev Pointer to BME68X device structure
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_init(struct bme68x_dev *dev);

/**
 * @brief Read raw sensor data from BME68X
 * 
 * @param dev Pointer to BME68X device structure
 * @param data Pointer to store the raw sensor data
 * @param n_data Pointer to store the number of valid data fields
 * @param op_mode Operating mode (forced, parallel, or sequential)
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_read_data(struct bme68x_dev *dev, struct bme68x_data *data, uint8_t *n_data, uint8_t op_mode);

/**
 * @brief Configure BME68X sensor settings
 * 
 * @param dev Pointer to BME68X device structure
 * @param conf Pointer to configuration structure
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_configure(struct bme68x_dev *dev, struct bme68x_conf *conf);

/**
 * @brief Configure BME68X heater settings
 * 
 * @param dev Pointer to BME68X device structure
 * @param op_mode Operating mode (forced, parallel, or sequential)
 * @param heatr_conf Pointer to heater configuration structure
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_set_heater_conf(struct bme68x_dev *dev, uint8_t op_mode, const struct bme68x_heatr_conf *heatr_conf);

/**
 * @brief Get current sensor settings
 * 
 * @param dev Pointer to BME68X device structure
 * @param conf Pointer to store current configuration
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_get_settings(struct bme68x_dev *dev, struct bme68x_conf *conf);

/**
 * @brief Set sensor operation mode
 * 
 * @param dev Pointer to BME68X device structure
 * @param op_mode Operation mode to set
 * @return int8_t BME68X_OK on success, error code otherwise
 */
int8_t sensor_api_set_op_mode(struct bme68x_dev *dev, uint8_t op_mode);

#endif // SENSOR_API_H 