#include "gap.h"
#include "common.h"

/* Private function declarations */
inline static void format_addr(char *addr_str, uint8_t addr[]);
static void start_advertising(void);

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static bool advertising = false;

/* Environmental Sensing Service UUID */
static const ble_uuid16_t gatt_svr_svc_temp_uuid = BLE_UUID16_INIT(0x181A);

/* Manufacturer-specific data structure - reduced size to avoid exceeding BLE limits */
struct __attribute__((packed)) {
    uint16_t company_id;      // 0x02E5 for Espressif
    uint8_t  data_type;       // Data type identifier (0x01)
    int16_t  temperature;     // Temperature value (scaled)
    uint8_t  humidity;        // Humidity value (scaled)
    uint8_t  pressure_msb;    // Pressure MSB
    uint8_t  pressure_lsb;    // Pressure LSB
} manufacturer_data = {
    .company_id = 0x02E5,     // Espressif's company ID
    .data_type = 0x01         // Custom data type
};

/* Private functions */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static void start_advertising(void) {
    /* Local variables */
    int rc = 0;
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};

    /* Set advertising flags */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    /* Set device appearance as a temperature sensor */
    fields.appearance = BLE_APPEARANCE_GENERIC_THERMOMETER;
    fields.appearance_is_present = 1;
    
    /* Set Environmental Sensing Service UUID */
    fields.uuids16 = &gatt_svr_svc_temp_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    /* Set manufacturer-specific data - must keep total under 31 bytes */
    fields.mfg_data = (uint8_t *)&manufacturer_data;
    fields.mfg_data_len = sizeof(manufacturer_data);
    
    /* Set advertisement fields */
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data, error code: %d", rc);
        return;
    }
    
    /* Set scan response fields with device name */
    rsp_fields.name = (uint8_t *)DEVICE_NAME;
    rsp_fields.name_len = strlen(DEVICE_NAME);
    rsp_fields.name_is_complete = 1;
    
    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response data, error code: %d", rc);
        // Continue anyway - not critical
    }

    /* Set non-connectable and general discoverable mode */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    /* Advertise at a moderate rate to save power (300-400ms interval) */
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL2_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL2_MAX;

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising, error code: %d", rc);
        return;
    }
    advertising = true;
    ESP_LOGI(TAG, "Advertising started successfully as \"%s\"!", DEVICE_NAME);
}

/* Public functions */
void adv_init(void) {
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Device does not have any available BT address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type, error code: %d", rc);
        return;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "Device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

void adv_update_sensor_data(t_a_h_data_t *sensor_data) {
    int rc = 0;
    
    if (!advertising) {
        ESP_LOGE(TAG, "Advertising not started");
        return;
    }
    
    /* Stop advertising to update data */
    rc = ble_gap_adv_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to stop advertising, error code: %d", rc);
        return;
    }
    
    /* Update manufacturer data with sensor readings - compressed format */
    manufacturer_data.temperature = (int16_t)(sensor_data->temperature * 100); // Scale by 100 to keep 2 decimal places
    manufacturer_data.humidity = (uint8_t)(sensor_data->humidity);             // Integer percentage is sufficient
    
    /* Encode pressure as MSB/LSB pair to save space */
    uint16_t pressure = (uint16_t)(sensor_data->pressure * 10);  // Convert to hPa/10 to fit in 16 bits
    manufacturer_data.pressure_msb = (pressure >> 8) & 0xFF;
    manufacturer_data.pressure_lsb = pressure & 0xFF;
    
    ESP_LOGI(TAG, "Updated sensor data: T=%.2f°C, H=%.0f%%, P=%.1fhPa", 
             sensor_data->temperature, sensor_data->humidity, sensor_data->pressure);
    
    /* Restart advertising with new data */
    start_advertising();
}

void adv_stop(void) {
    int rc = 0;
    
    if (!advertising) {
        return;
    }
    
    rc = ble_gap_adv_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to stop advertising, error code: %d", rc);
        return;
    }
    
    advertising = false;
    ESP_LOGI(TAG, "Advertising stopped");
}

int gap_init(void) {
    /* Local variables */
    int rc = 0;

    /* Initialize GAP service */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name to %s, error code: %d",
                 DEVICE_NAME, rc);
        return rc;
    }
    
    /* Set device appearance as a temperature sensor */
    rc = ble_svc_gap_device_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device appearance, error code: %d", rc);
        // Not critical, continue anyway
    }

    return rc;
} 