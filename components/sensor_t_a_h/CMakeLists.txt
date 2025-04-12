idf_component_register(SRCS "sensor_t_a_h.c" "sensor_api.c" "common.c" "./BME680_SensorAPI/bme68x.c"
                    INCLUDE_DIRS "." "include" "./BME680_SensorAPI"
                    PRIV_INCLUDE_DIRS "./priv_include"
                    PRIV_REQUIRES esp_driver_i2c esp_driver_spi freertos esp_timer)
