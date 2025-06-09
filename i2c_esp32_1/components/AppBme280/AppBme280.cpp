#include "AppBme280.hpp"

AppBme280::AppBme280(const std::string name) : name_(name) {}

esp_err_t AppBme280::init(bme280_config_t config) {
    static const char* tag = "AppBme280::init";
    config_ = config;
    ESP_ERROR_CHECK(config_.i2c->add_device(name_, config.address, config.freq));

    // try reading the ID register
    ESP_ERROR_CHECK(read_reg(0xd0, 1));

    ESP_LOGD(tag, "0x%02x", reg_data_[0]);
    return ESP_OK;
}

esp_err_t AppBme280::read_reg(const uint8_t reg_addr, uint8_t len) {
    return config_.i2c->read_bytes(name_, reg_addr, len, reg_data_);
}
