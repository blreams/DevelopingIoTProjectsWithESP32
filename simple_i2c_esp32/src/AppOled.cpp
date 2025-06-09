#include "esp_log.h"
#include "AppOled.hpp"
//#include "esp_ssd1306.h"

AppOled::AppOled(uint8_t address, AppI2c* i2c) : address_(address), i2c_(i2c) {};

/*
esp_err_t AppOled::init() {
    static i2c_master_bus_config_t i2c_master_bus_config = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    static i2c_master_bus_handle_t i2c_master_bus;
    static i2c_ssd1306_config_t config = {
        .i2c_device_address = address_;
        .i2c_scl_speed_hz = 100000,
        .width = 128,
        .height = 64,
        .wise = SSD1306_TOP_TO_BOTTOM
    };
    static i2c_ssd1306_handle_t i2c_ssd1306;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus));
    ESP_ERROR_CHECK(i2c_ssd1306_init(i2c_, &config, &i2c_ssd1306));
    return ESP_OK;
}
*/