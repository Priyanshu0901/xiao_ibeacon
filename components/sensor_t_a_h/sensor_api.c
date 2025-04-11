#include "sensor_api.h"
#include "esp_log.h"
#include "common.h"

static const char *TAG = "SENSOR_API";

int8_t sensor_api_init(struct bme280_dev *dev)
{
    int8_t rslt;
    
    /* Interface selection - use function from common.h */
#if defined(CONFIG_BME280_USE_SPI)
    rslt = bme280_interface_selection(dev, BME280_SPI_INTF);
    ESP_LOGI(TAG, "Using SPI interface for BME280");
#elif defined(CONFIG_BME280_USE_I2C)
    rslt = bme280_interface_selection(dev, BME280_I2C_INTF);
    ESP_LOGI(TAG, "Using I2C interface for BME280");
#else
    #error "No BME280 interface selected in Kconfig. Please select either I2C or SPI."
#endif

    bme280_error_codes_print_result("bme280_interface_selection", rslt);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BME280 interface");
        return rslt;
    }

    /* Initialize the sensor */
    rslt = bme280_init(dev);
    bme280_error_codes_print_result("bme280_init", rslt);
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BME280 sensor");
        return rslt;
    }

    return BME280_OK;
}

int8_t sensor_api_read_data(struct bme280_dev *dev, struct bme280_data *data)
{
    int8_t rslt;
    
    /* Read the sensor data */
    rslt = bme280_get_sensor_data(BME280_ALL, data, dev);
    bme280_error_codes_print_result("bme280_get_sensor_data", rslt);
    
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return rslt;
    }

    return BME280_OK;
}

int8_t sensor_api_configure(struct bme280_dev *dev, struct bme280_settings *settings)
{
    int8_t rslt;
    
    /* Configure the sensor settings */
    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, settings, dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);
    
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to configure sensor settings");
        return rslt;
    }

    return BME280_OK;
}

int8_t sensor_api_get_settings(struct bme280_dev *dev, struct bme280_settings *settings)
{
    int8_t rslt;
    
    /* Get current sensor settings */
    rslt = bme280_get_sensor_settings(settings, dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);
    
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to get sensor settings");
        return rslt;
    }

    return BME280_OK;
}

int8_t sensor_api_set_power_mode(struct bme280_dev *dev, uint8_t mode)
{
    int8_t rslt;
    
    /* Set sensor power mode */
    rslt = bme280_set_sensor_mode(mode, dev);
    bme280_error_codes_print_result("bme280_set_sensor_mode", rslt);
    
    if (rslt != BME280_OK)
    {
        ESP_LOGE(TAG, "Failed to set sensor power mode");
        return rslt;
    }

    return BME280_OK;
} 