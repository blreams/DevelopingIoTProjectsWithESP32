#pragma once

#include <string>
#include "AppI2cMaster.hpp"

struct bme280_config_t {
    uint8_t address;
    uint32_t freq;
    uint32_t read_latency;
    AppI2cMaster* i2c;
};

class AppBme280 {
public:
    explicit AppBme280(const std::string name);
    esp_err_t init(bme280_config_t config);
    esp_err_t read_reg(const uint8_t reg_addr, const uint8_t len);

private:
    std::string name_;
    bme280_config_t config_;
    uint8_t reg_data_[32] = {0};
};