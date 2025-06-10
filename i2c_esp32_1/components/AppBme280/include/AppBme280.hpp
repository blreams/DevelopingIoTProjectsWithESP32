#pragma once

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme_defs.hpp"
#include "AppGpioOut.hpp"
#include "AppI2cMaster.hpp"

struct bme280_calibration_data_t {
    uint16_t dig_t1 = 0;
    int16_t dig_t2 = 0;
    int16_t dig_t3 = 0;
    uint16_t dig_p1 = 0;
    int16_t dig_p2 = 0;
    int16_t dig_p3 = 0;
    int16_t dig_p4 = 0;
    int16_t dig_p5 = 0;
    int16_t dig_p6 = 0;
    int16_t dig_p7 = 0;
    int16_t dig_p8 = 0;
    int16_t dig_p9 = 0;
    uint8_t dig_h1 = 0;
    int16_t dig_h2 = 0;
    uint8_t dig_h3 = 0;
    int16_t dig_h4 = 0;
    int16_t dig_h5 = 0;
    int8_t dig_h6 = 0;
    int32_t t_fine = 0;
};

struct bme280_dev_t {
    uint8_t chip_id = 0;
    int8_t intf_rslt = 0;
    struct bme280_calibration_data_t calibration_data;
};

struct bme280_config_t {
    uint8_t address;
    uint32_t freq;
    uint32_t read_latency;
    AppI2cMaster* i2c;
};

class AppBme280 {
public:
    explicit AppBme280(const std::string name, gpio_num_t trigger_gpio=GPIO_NUM_32);
    bme_err_t init(bme280_config_t config);
    bme_err_t read_reg(const uint8_t reg_addr, const uint8_t len);
    bme_err_t write_reg(const uint8_t reg_addr, const uint8_t len, std::vector<uint8_t>& write_data);
    bme_err_t soft_reset();

private:
    std::string name_;
    bme280_config_t config_;
    std::vector<uint8_t> reg_data_;
    std::vector<uint8_t> write_data_;
    bme280_dev_t dev_;

    gpio_num_t trigger_gpio_;
    AppGpioOut trigger_;
};