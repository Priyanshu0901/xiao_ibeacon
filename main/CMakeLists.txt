idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES sensor_t_a_h ble_beacon power_manager spi_flash nvs_flash)
