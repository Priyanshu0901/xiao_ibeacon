#ifndef COMMON_H
#define COMMON_H

/* Includes */
/* STD APIs */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ESP APIs */
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* FreeRTOS APIs */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* Defines */
#define DEVICE_NAME "Volcatec_TempSensor"
#define TAG "BLE_Beacon"

/* BLE Appearance Values */
#define BLE_APPEARANCE_GENERIC_THERMOMETER 0x0300

/* Sensor data structure */
#include "sensor_t_a_h.h"

#endif // COMMON_H 