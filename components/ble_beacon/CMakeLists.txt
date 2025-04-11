idf_component_register(
    SRCS "ble_beacon.c" "gap.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES bt
    REQUIRES sensor_t_a_h nvs_flash
)
