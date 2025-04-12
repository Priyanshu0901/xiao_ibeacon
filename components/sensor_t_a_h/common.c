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
#include "bme68x.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sdkconfig.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME68X_TAG "BME68X"
#define BME68X_I2C_TIMEOUT_MS 100 /* 100 ms timeout for I2C operations */

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
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME68X_USE_I2C
    esp_err_t ret;
    
    /* First write the register address */
    ret = i2c_master_transmit(i2c_dev, &reg_addr, 1, BME68X_I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME68X_TAG, "I2C write address failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    /* Then read the data */
    ret = i2c_master_receive(i2c_dev, reg_data, length, BME68X_I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME68X_TAG, "I2C read data failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

/*!
 * I2C write function for ESP-IDF v5.4+
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME68X_USE_I2C
    esp_err_t ret;
    uint8_t *write_buf = malloc(length + 1);
    
    if (write_buf == NULL) {
        ESP_LOGE(BME68X_TAG, "Failed to allocate memory for I2C write buffer");
        return BME68X_E_COM_FAIL;
    }
    
    /* Prepare write buffer with register address as first byte */
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, reg_data, length);

    /* Write data to device - use i2c_dev, not intf_ptr */
    ret = i2c_master_transmit(i2c_dev, write_buf, length + 1, BME68X_I2C_TIMEOUT_MS);
    
    free(write_buf);
    
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME68X_TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }

    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

/*!
 * SPI read function for ESP-IDF
 */
BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME68X_USE_SPI
    esp_err_t ret;
    
    // BME68X requires the MSB of the register address to be set for read operations
    uint8_t tx_addr = reg_addr | 0x80;
    uint8_t *tx_buffer = malloc(length + 1);
    uint8_t *rx_buffer = malloc(length + 1);
    
    if (tx_buffer == NULL || rx_buffer == NULL) {
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        ESP_LOGE(BME68X_TAG, "Failed to allocate memory for SPI buffers");
        return BME68X_E_COM_FAIL;
    }
    
    // Fill TX buffer: first byte is address, rest are zeros for read operation
    tx_buffer[0] = tx_addr;
    memset(tx_buffer + 1, 0, length);
    
    // Create transaction
    spi_transaction_t t = {
        .length = 8 * (length + 1),  // bits
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };
    
    // Execute transaction
    ret = spi_device_transmit(spi_handle, &t);
    
    // Copy result data (skip first byte which was address)
    if (ret == ESP_OK) {
        memcpy(reg_data, rx_buffer + 1, length);
    }
    
    free(tx_buffer);
    free(rx_buffer);
    
    if (ret != ESP_OK) {
        ESP_LOGE(BME68X_TAG, "SPI read transaction failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

/*!
 * SPI write function for ESP-IDF
 */
BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#ifdef CONFIG_BME68X_USE_SPI
    esp_err_t ret;
    
    // For BME68X, clear MSB (0x7F) for write operations
    uint8_t *tx_buffer = malloc(length + 1);
    
    if (tx_buffer == NULL) {
        ESP_LOGE(BME68X_TAG, "Failed to allocate memory for SPI write buffer");
        return BME68X_E_COM_FAIL;
    }
    
    // First byte is register address (with MSB cleared for write operation)
    tx_buffer[0] = reg_addr & 0x7F;
    memcpy(tx_buffer + 1, reg_data, length);
    
    // Create transaction
    spi_transaction_t t = {
        .length = 8 * (length + 1),  // bits
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL
    };
    
    // Execute transaction
    ret = spi_device_transmit(spi_handle, &t);
    
    free(tx_buffer);
    
    if (ret != ESP_OK) {
        ESP_LOGE(BME68X_TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

/*!
 * Delay function for ESP-IDF
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme68x_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME68X_OK)
    {
        ESP_LOGE(BME68X_TAG, "%s\t", api_name);

        switch (rslt)
        {
        case BME68X_E_NULL_PTR:
            ESP_LOGE(BME68X_TAG, "Error [%d] : Null pointer error.", rslt);
            ESP_LOGE(BME68X_TAG,
                     "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.");
            break;

        case BME68X_E_COM_FAIL:
            ESP_LOGE(BME68X_TAG, "Error [%d] : Communication failure error.", rslt);
            ESP_LOGE(BME68X_TAG,
                     "It occurs due to read/write operation failure and also due to power failure during communication");
            break;

        case BME68X_E_DEV_NOT_FOUND:
            ESP_LOGE(BME68X_TAG, "Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read",
                     rslt);
            break;

        case BME68X_E_INVALID_LENGTH:
            ESP_LOGE(BME68X_TAG, "Error [%d] : Invalid length error. It occurs when write is done with invalid length", rslt);
            break;

        default:
            ESP_LOGE(BME68X_TAG, "Error [%d] : Unknown error code", rslt);
            break;
        }
    }
}

/*!
 *  @brief Function to initialize the I2C interface using latest ESP-IDF v5.4+ API
 */
static esp_err_t bme68x_i2c_init(void)
{
#ifdef CONFIG_BME68X_USE_I2C
    esp_err_t ret;

    // I2C master configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_BME68X_I2C_PORT,
        .scl_io_num = CONFIG_BME68X_I2C_SCL_PIN,
        .sda_io_num = CONFIG_BME68X_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Create I2C master bus
    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME68X_TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // I2C device configuration
    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_BME68X_I2C_ADDR,
        .scl_speed_hz = CONFIG_BME68X_I2C_CLOCK_SPEED,
    };

    // Add device to the I2C bus
    ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_config, &i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BME68X_TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    #ifdef DEBUG_BME
    ret = i2c_master_probe(bus_handle, CONFIG_BME68X_I2C_ADDR, 100);
    if (ESP_OK == ret)
    {
        printf("Initalized device : 0x%0x\n", CONFIG_BME68X_I2C_ADDR);
    }
    else
    {
        printf("Initalized failed for device : 0x%0x \t0x%0x\n", CONFIG_BME68X_I2C_ADDR, ret);
    }
    #endif
    
    return ESP_OK;
#else
    ESP_LOGW(BME68X_TAG, "I2C interface not enabled in config");
    return ESP_FAIL;
#endif
}

/*!
 *  @brief Function to initialize the SPI interface
 */
static esp_err_t bme68x_spi_init(void)
{
#ifdef CONFIG_BME68X_USE_SPI
    esp_err_t ret;
    
    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_BME68X_SPI_MISO_PIN,
        .mosi_io_num = CONFIG_BME68X_SPI_MOSI_PIN,
        .sclk_io_num = CONFIG_BME68X_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,  // Max BME68X transaction size
    };

    // SPI device configuration - standard full-duplex mode
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CONFIG_BME68X_SPI_CLOCK_SPEED,
        .mode = 0,                // BME68X works in mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = CONFIG_BME68X_SPI_CS_PIN,
        .queue_size = 7,
        .command_bits = 0,        // No command bits for BME68X
        .address_bits = 0,        // No address bits for BME68X
        .dummy_bits = 0,          // No dummy bits for BME68X
        .cs_ena_pretrans = 5,     // Assert CS 5 cycles before transaction
        .cs_ena_posttrans = 5,    // Keep CS active 5 cycles after transaction
        .flags = 0,               // Regular full-duplex mode
        // Important: BME68X uses positive CS logic - active high
        .pre_cb = NULL,           // No pre-transfer callback
        .post_cb = NULL,          // No post-transfer callback
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(CONFIG_BME68X_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(BME68X_TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add the BME68X device to the SPI bus
    ret = spi_bus_add_device(CONFIG_BME68X_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(BME68X_TAG, "SPI device addition failed: %s", esp_err_to_name(ret));
        spi_bus_free(CONFIG_BME68X_SPI_HOST);
        return ret;
    }

    ESP_LOGI(BME68X_TAG, "SPI interface initialized successfully with CS pin: %d", CONFIG_BME68X_SPI_CS_PIN);
    return ESP_OK;
#else
    ESP_LOGW(BME68X_TAG, "SPI interface not enabled in config");
    return ESP_FAIL;
#endif
}

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 *
 *  @param[in] dev    : Structure instance of bme68x_dev
 *  @param[in] intf   : Interface selection parameter
 *                          For I2C : BME68X_I2C_INTF
 *                          For SPI : BME68X_SPI_INTF
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure
 */
int8_t bme68x_interface_init(struct bme68x_dev *dev, uint8_t intf)
{
    int8_t rslt = BME68X_OK;
    esp_err_t esp_rslt;

    if (dev == NULL)
    {
        return BME68X_E_NULL_PTR;
    }

    /* Initialize I2C or SPI based on requested interface */
    if (intf == BME68X_I2C_INTF)
    {
#ifdef CONFIG_BME68X_USE_I2C
        ESP_LOGI(BME68X_TAG, "Initializing BME68X with I2C interface");
        
        /* Initialize I2C */
        esp_rslt = bme68x_i2c_init();
        if (esp_rslt != ESP_OK)
        {
            return BME68X_E_COM_FAIL;
        }
        
        /* Set function pointers */
        dev->read = bme68x_i2c_read;
        dev->write = bme68x_i2c_write;
        dev->intf = BME68X_I2C_INTF;
        dev_addr = CONFIG_BME68X_I2C_ADDR;
#else
        ESP_LOGE(BME68X_TAG, "I2C interface requested but not enabled in config");
        return BME68X_E_COM_FAIL;
#endif
    }
    else if (intf == BME68X_SPI_INTF)
    {
#ifdef CONFIG_BME68X_USE_SPI
        ESP_LOGI(BME68X_TAG, "Initializing BME68X with SPI interface");
        
        /* Initialize SPI */
        esp_rslt = bme68x_spi_init();
        if (esp_rslt != ESP_OK)
        {
            return BME68X_E_COM_FAIL;
        }
        
        /* Set function pointers */
        dev->read = bme68x_spi_read;
        dev->write = bme68x_spi_write;
        dev->intf = BME68X_SPI_INTF;
        dev_addr = CONFIG_BME68X_SPI_CS_PIN;
#else
        ESP_LOGE(BME68X_TAG, "SPI interface requested but not enabled in config");
        return BME68X_E_COM_FAIL;
#endif
    }
    else
    {
        ESP_LOGE(BME68X_TAG, "Invalid interface selection: %d", intf);
        return BME68X_E_COM_FAIL;
    }
    
    /* Set interface parameters */
    dev->intf_ptr = &dev_addr;
    dev->delay_us = bme68x_delay_us;
    dev->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    
    return rslt;
}

/*!
 *  @brief Function deinitializes the interfaces.
 */
void bme68x_deinit(uint8_t intf)
{
    if (intf == BME68X_I2C_INTF)
    {
#ifdef CONFIG_BME68X_USE_I2C
        if (i2c_dev)
        {
            esp_err_t ret;
            i2c_master_bus_handle_t bus_handle;
            ret = i2c_master_bus_rm_device(i2c_dev);
            if (ret != ESP_OK)
            {
                ESP_LOGE(BME68X_TAG, "Error removing I2C device: %s", esp_err_to_name(ret));
            }

            ret = i2c_master_get_bus_handle(CONFIG_BME68X_I2C_PORT, &bus_handle);
            if (ret == ESP_OK && bus_handle)
            {
                i2c_del_master_bus(bus_handle);
            }

            i2c_dev = NULL;
        }
#endif
    }
    else if (intf == BME68X_SPI_INTF)
    {
#ifdef CONFIG_BME68X_USE_SPI
        if (spi_handle)
        {
            spi_bus_remove_device(spi_handle);
            spi_bus_free(CONFIG_BME68X_SPI_HOST);
            spi_handle = NULL;
        }
#endif
    }
}

/*!
 *  @brief Function deinitializes coines platform.
 */
void bme68x_coines_deinit(void)
{
}