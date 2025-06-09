#pragma once

#include <vector>
#include <string>
#include <rom/ets_sys.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_PORT I2C_NUM_0
#define I2C_MIN_FREQ_HZ 100000
#define I2C_MAX_FREQ_HZ 400000

struct DeviceInfo {
    std::string name;
    uint8_t address;
    uint32_t freq;
    uint32_t read_latency;
    float scl_period;
    i2c_master_dev_handle_t handle;
};

class AppI2cMaster {
public:
    explicit AppI2cMaster(gpio_num_t scl=GPIO_NUM_22, gpio_num_t sda=GPIO_NUM_21);
    esp_err_t init();
    esp_err_t add_device(std::string name, uint8_t address, uint32_t freq, uint32_t read_latency=100);
    DeviceInfo get_device_by_name(const std::string& name);
    esp_err_t read_bytes(const std::string name, uint8_t reg, uint8_t len, uint8_t* buffer);
private:
    gpio_num_t scl_;
    gpio_num_t sda_;
    std::vector<DeviceInfo> devices_;
    i2c_master_bus_handle_t bus_handle_;
};