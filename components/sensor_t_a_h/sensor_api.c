#include "sensor_api.h"
#include "esp_log.h"
#include "common.h"

static const char *TAG = "SENSOR_API";

int8_t sensor_api_init(struct bme68x_dev *dev)
{
    int8_t rslt;
    
    /* Interface selection - use function from common.h */
#if defined(CONFIG_BME68X_USE_SPI)
    rslt = bme68x_interface_init(dev, BME68X_SPI_INTF);
    ESP_LOGI(TAG, "Using SPI interface for BME68X");
#elif defined(CONFIG_BME68X_USE_I2C)
    rslt = bme68x_interface_init(dev, BME68X_I2C_INTF);
    ESP_LOGI(TAG, "Using I2C interface for BME68X");
#else
    #error "No BME68X interface selected in Kconfig. Please select either I2C or SPI."
#endif

    bme68x_error_codes_print_result("bme68x_interface_init", rslt);
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BME68X interface");
        return rslt;
    }

    /* Initialize the sensor */
    rslt = bme68x_init(dev);
    bme68x_error_codes_print_result("bme68x_init", rslt);
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BME68X sensor");
        return rslt;
    }

    return BME68X_OK;
}

int8_t sensor_api_read_data(struct bme68x_dev *dev, struct bme68x_data *data, uint8_t *n_data, uint8_t op_mode)
{
    int8_t rslt;
    
    /* Read the sensor data */
    rslt = bme68x_get_data(op_mode, data, n_data, dev);
    bme68x_error_codes_print_result("bme68x_get_data", rslt);
    
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return rslt;
    }

    return BME68X_OK;
}

int8_t sensor_api_configure(struct bme68x_dev *dev, struct bme68x_conf *conf)
{
    int8_t rslt;
    
    /* Configure the sensor settings */
    rslt = bme68x_set_conf(conf, dev);
    bme68x_error_codes_print_result("bme68x_set_conf", rslt);
    
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to configure sensor settings");
        return rslt;
    }

    return BME68X_OK;
}

int8_t sensor_api_set_heater_conf(struct bme68x_dev *dev, uint8_t op_mode, const struct bme68x_heatr_conf *heatr_conf)
{
    int8_t rslt;
    
    /* Configure heater settings */
    rslt = bme68x_set_heatr_conf(op_mode, heatr_conf, dev);
    bme68x_error_codes_print_result("bme68x_set_heatr_conf", rslt);
    
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to configure heater settings");
        return rslt;
    }

    return BME68X_OK;
}

int8_t sensor_api_get_settings(struct bme68x_dev *dev, struct bme68x_conf *conf)
{
    int8_t rslt;
    
    /* Get current sensor settings */
    rslt = bme68x_get_conf(conf, dev);
    bme68x_error_codes_print_result("bme68x_get_conf", rslt);
    
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to get sensor settings");
        return rslt;
    }

    return BME68X_OK;
}

int8_t sensor_api_set_op_mode(struct bme68x_dev *dev, uint8_t op_mode)
{
    int8_t rslt;
    
    /* Set sensor operation mode */
    rslt = bme68x_set_op_mode(op_mode, dev);
    bme68x_error_codes_print_result("bme68x_set_op_mode", rslt);
    
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(TAG, "Failed to set sensor operation mode");
        return rslt;
    }

    return BME68X_OK;
} 