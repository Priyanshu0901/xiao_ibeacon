/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Modified for ESP-IDF framework v5.4+
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "bme280.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sdkconfig.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME280_TAG "BME280"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/* I2C master handle */
static i2c_master_dev_handle_t i2c_dev = NULL;

/* SPI handle */
static spi_device_handle_t spi_handle;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function for ESP-IDF v5.4+
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME280_USE_I2C
    esp_err_t ret;
    uint8_t write_cmd[1] = {reg_addr};

    i2c_master_transmit_receive_t i2c_cmd = {
        .rx_buffer = reg_data,
        .rx_size = length,
        .slave_addr = CONFIG_BME280_I2C_ADDR,
        .tx_buffer = write_cmd,
        .tx_size = 1,
    };

    ret = i2c_master_transmit_receive(intf_ptr, &i2c_cmd, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME280_TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
#else
    return BME280_E_COMM_FAIL;
#endif
}

/*!
 * I2C write function for ESP-IDF v5.4+
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME280_USE_I2C
    esp_err_t ret;
    uint8_t write_buf[20]; // Max length should be sufficient

    /* Ensure we don't exceed buffer size */
    if (length >= sizeof(write_buf) - 1)
    {
        ESP_LOGE(BME280_TAG, "I2C write buffer overflow");
        return BME280_E_INVALID_LEN;
    }

    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, reg_data, length);

    ret = i2c_master_transmit(intf_ptr, write_buf, length + 1, CONFIG_BME280_I2C_ADDR, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME280_TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
#else
    return BME280_E_COMM_FAIL;
#endif
}

/*!
 * SPI read function for ESP-IDF
 */
BME280_INTF_RET_TYPE bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME280_USE_SPI
    esp_err_t ret;
    
    // BME280 requires the MSB of the register address to be set for read operations
    uint8_t tx_addr = reg_addr | 0x80;
    
    // For BME280, we can actually do the read in a single transaction
    // First prepare a buffer for TX data
    uint8_t tx_buffer[length + 1];
    memset(tx_buffer, 0, sizeof(tx_buffer));  // Fill with zeros for dummy bytes during read
    tx_buffer[0] = tx_addr;                  // First byte is address with read bit set
    
    // Prepare a buffer for RX data
    uint8_t rx_buffer[length + 1];
    memset(rx_buffer, 0, sizeof(rx_buffer));
    
    // Create a single transaction - SPI driver handles CS automatically
    spi_transaction_t t = {
        .length = 8 * (length + 1),  // Total length in bits (address + data)
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .flags = 0
    };
    
    // Execute transaction
    ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(BME280_TAG, "SPI read transaction failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }
    
    // Copy received data (skip the first byte which was used for sending address)
    memcpy(reg_data, rx_buffer + 1, length);
    
    return BME280_OK;
#else
    return BME280_E_COMM_FAIL;
#endif
}

/*!
 * SPI write function for ESP-IDF
 */
BME280_INTF_RET_TYPE bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME280_USE_SPI
    esp_err_t ret;
    
    // For BME280, clear MSB (0x7F) for write operations
    uint8_t tx_buffer[16]; // Maximum buffer size (adjust if needed)
    
    // Check if buffer size is sufficient
    if (length + 1 > sizeof(tx_buffer)) {
        ESP_LOGE(BME280_TAG, "SPI write buffer overflow");
        return BME280_E_INVALID_LEN;
    }
    
    // First byte is register address (with MSB cleared for write operation)
    tx_buffer[0] = reg_addr & 0x7F;
    memcpy(&tx_buffer[1], reg_data, length);
    
    // Create a single transaction for the entire write operation
    // SPI driver handles CS assertion/deassertion automatically
    spi_transaction_t t = {
        .length = (length + 1) * 8,  // Register address + data in bits
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL,  // No need to receive during write
        .flags = 0
    };
    
    // Execute the transaction
    ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(BME280_TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }
    
    return BME280_OK;
#else
    return BME280_E_COMM_FAIL;
#endif
}

/*!
 * Delay function for ESP-IDF
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        ESP_LOGE(BME280_TAG, "%s\t", api_name);

        switch (rslt)
        {
        case BME280_E_NULL_PTR:
            ESP_LOGE(BME280_TAG, "Error [%d] : Null pointer error.", rslt);
            ESP_LOGE(BME280_TAG,
                     "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.");
            break;

        case BME280_E_COMM_FAIL:
            ESP_LOGE(BME280_TAG, "Error [%d] : Communication failure error.", rslt);
            ESP_LOGE(BME280_TAG,
                     "It occurs due to read/write operation failure and also due to power failure during communication");
            break;

        case BME280_E_DEV_NOT_FOUND:
            ESP_LOGE(BME280_TAG, "Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read",
                     rslt);
            break;

        case BME280_E_INVALID_LEN:
            ESP_LOGE(BME280_TAG, "Error [%d] : Invalid length error. It occurs when write is done with invalid length", rslt);
            break;

        default:
            ESP_LOGE(BME280_TAG, "Error [%d] : Unknown error code", rslt);
            break;
        }
    }
}

/*!
 *  @brief Function to initialize the I2C interface using latest ESP-IDF v5.4+ API
 */
static esp_err_t bme280_i2c_init(void)
{
#ifdef CONFIG_BME280_USE_I2C
    esp_err_t ret;

    // I2C master configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_BME280_I2C_PORT,
        .scl_io_num = CONFIG_BME280_I2C_SCL_PIN,
        .sda_io_num = CONFIG_BME280_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Create I2C master bus
    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME280_TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // I2C device configuration
    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_BME280_I2C_ADDR,
        .scl_speed_hz = CONFIG_BME280_I2C_CLOCK_SPEED,
    };

    // Add device to the I2C bus
    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_config, &i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME280_TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    #ifdef DEBUG_BME
    ret = i2c_master_probe(bus_handle, CONFIG_BME280_I2C_ADDR, 100);
    if (ESP_OK == ret)
    {
        printf("Initalized device : 0x%0x\n", CONFIG_BME280_I2C_ADDR);
    }
    else
    {
        printf("Initalized failed for device : 0x%0x \t0x%0x\n", CONFIG_BME280_I2C_ADDR, ret);
    }
    #endif
    
    return ESP_OK;
#else
    ESP_LOGW(BME280_TAG, "I2C interface not enabled in config");
    return ESP_FAIL;
#endif
}

/*!
 *  @brief Function to initialize the SPI interface
 */
static esp_err_t bme280_spi_init(void)
{
#ifdef CONFIG_BME280_USE_SPI
    esp_err_t ret;
    
    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_BME280_SPI_MISO_PIN,
        .mosi_io_num = CONFIG_BME280_SPI_MOSI_PIN,
        .sclk_io_num = CONFIG_BME280_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,  // Max BME280 transaction size
    };

    // SPI device configuration - standard full-duplex mode
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CONFIG_BME280_SPI_CLOCK_SPEED,
        .mode = 0,                // BME280 works in mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = CONFIG_BME280_SPI_CS_PIN,
        .queue_size = 7,
        .command_bits = 0,        // No command bits for BME280
        .address_bits = 0,        // No address bits for BME280
        .dummy_bits = 0,          // No dummy bits for BME280
        .cs_ena_pretrans = 5,     // Assert CS 5 cycles before transaction
        .cs_ena_posttrans = 5,    // Keep CS active 5 cycles after transaction
        .flags = 0,               // Regular full-duplex mode
        // Important: BME280 uses positive CS logic - active high
        .pre_cb = NULL,           // No pre-transfer callback
        .post_cb = NULL,          // No post-transfer callback
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(CONFIG_BME280_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(BME280_TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add the BME280 device to the SPI bus
    ret = spi_bus_add_device(CONFIG_BME280_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(BME280_TAG, "SPI device addition failed: %s", esp_err_to_name(ret));
        spi_bus_free(CONFIG_BME280_SPI_HOST);
        return ret;
    }

    ESP_LOGI(BME280_TAG, "SPI interface initialized successfully with CS pin: %d", CONFIG_BME280_SPI_CS_PIN);
    return ESP_OK;
#else
    ESP_LOGW(BME280_TAG, "SPI interface not enabled in config");
    return ESP_FAIL;
#endif
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_interface_selection(struct bme280_dev *dev, uint8_t intf)
{
    int8_t rslt = BME280_OK;
    esp_err_t ret;

    if (dev != NULL)
    {
        /* Bus configuration : I2C (default) */
        if (intf == BME280_I2C_INTF)
        {
#ifdef CONFIG_BME280_USE_I2C
            ESP_LOGI(BME280_TAG, "Initializing I2C Interface");

            dev_addr = CONFIG_BME280_I2C_ADDR;
            dev->intf_ptr = (void *)i2c_dev; // Pass I2C handle as intf_ptr
            dev->read = bme280_i2c_read;
            dev->write = bme280_i2c_write;
            dev->intf = BME280_I2C_INTF;

            /* Initialize I2C */
            ret = bme280_i2c_init();
            if (ret != ESP_OK)
            {
                ESP_LOGE(BME280_TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
                return BME280_E_COMM_FAIL;
            }
#else
            ESP_LOGE(BME280_TAG, "I2C interface requested but not enabled in config");
            return BME280_E_COMM_FAIL;
#endif
        }
        /* Bus configuration : SPI */
        else if (intf == BME280_SPI_INTF)
        {
#ifdef CONFIG_BME280_USE_SPI
            ESP_LOGI(BME280_TAG, "Initializing SPI Interface");

            dev_addr = CONFIG_BME280_SPI_CS_PIN;
            dev->read = bme280_spi_read;
            dev->write = bme280_spi_write;
            dev->intf = BME280_SPI_INTF;

            /* Initialize SPI */
            ret = bme280_spi_init();
            if (ret != ESP_OK)
            {
                ESP_LOGE(BME280_TAG, "SPI initialization failed: %s", esp_err_to_name(ret));
                return BME280_E_COMM_FAIL;
            }
#else
            ESP_LOGE(BME280_TAG, "SPI interface requested but not enabled in config");
            return BME280_E_COMM_FAIL;
#endif
        }

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bme280_delay_us;

        /* Wait for sensor to power up */
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Function deinitializes the interfaces.
 */
void bme280_deinit(uint8_t intf)
{
    if (intf == BME280_I2C_INTF)
    {
#ifdef CONFIG_BME280_USE_I2C
        if (i2c_dev)
        {
            esp_err_t ret;
            i2c_master_bus_handle_t bus_handle;
            ret = i2c_master_bus_rm_device(i2c_dev);
            if (ret != ESP_OK)
            {
                ESP_LOGE(BME280_TAG, "Error removing I2C device: %s", esp_err_to_name(ret));
            }

            ret = i2c_master_get_bus_handle(CONFIG_BME280_I2C_PORT, &bus_handle);
            if (ret == ESP_OK && bus_handle)
            {
                i2c_del_master_bus(bus_handle);
            }

            i2c_dev = NULL;
        }
#endif
    }
    else if (intf == BME280_SPI_INTF)
    {
#ifdef CONFIG_BME280_USE_SPI
        if (spi_handle)
        {
            spi_bus_remove_device(spi_handle);
            spi_bus_free(CONFIG_BME280_SPI_HOST);
            spi_handle = NULL;
        }
#endif
    }
}

/*!
 *  @brief Function deinitializes coines platform.
 */
void bme280_coines_deinit(void)
{
}