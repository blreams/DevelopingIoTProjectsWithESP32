#include <iostream>
#include "AppI2c.hpp"
#include "esp_log.h"

AppI2c::AppI2c(gpio_num_t scl, gpio_num_t sda) : scl_(scl), sda_(sda) {}

esp_err_t AppI2c::init() {
    char IITAG[] = "I2C_INIT";
    ESP_LOGD(IITAG, "SCL GPIO = (%d)", scl_);
    ESP_LOGD(IITAG, "SDA GPIO = (%d)", sda_);
    initialized = false;
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_;
    conf.scl_io_num = scl_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(
        i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
        )
    );
    initialized = true;
    return ESP_OK;
}

int8_t AppI2c::user_i2c_read(
    uint8_t reg_addr,
    uint8_t* data,
    uint32_t len,
    void* intf_ptr
) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : 1;
}

int8_t AppI2c::user_i2c_write(
    uint8_t reg_addr,
    const uint8_t* data,
    uint32_t len,
    void* intf_ptr
) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : 1;
}

void AppI2c::user_delay_us(uint32_t period, void* intf_ptr) {
    ets_delay_us(period);
}

void AppI2c::scan_i2c_bus() {
    char ISTAG[] = "I2C_SCAN";
    ESP_LOGI(ISTAG, "Starting I2C scan...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI(ISTAG, "Found device at 0x%02X", addr);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGD(ISTAG, "Address 0x%02X: %s", addr, esp_err_to_name(err));
        }
    }
    ESP_LOGI(ISTAG, "I2C scan complete.");

}