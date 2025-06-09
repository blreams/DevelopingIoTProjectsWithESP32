#include <stdexcept>
#include <algorithm>
#include "AppI2cMaster.hpp"
#include "esp_log.h"


AppI2cMaster::AppI2cMaster(gpio_num_t scl, gpio_num_t sda) : scl_(scl), sda_(sda) {};

esp_err_t AppI2cMaster::init() {
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = sda_,
        .scl_io_num = scl_,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 1,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle_));
    return ESP_OK;
}

esp_err_t AppI2cMaster::add_device(std::string name, uint8_t address, uint32_t freq, uint32_t read_latency) {
/*
Attach peripheral device to bus.
  name: unique string
  address: i2c bus address
  freq: i2c scl frequency in hz (can be 100K, 400K)
  read_latency_overhead: extra delay to add before read data is available

There are two parts to the read latency. There is the raw scl period multiplied
by the number of bytes being read. To that we add the read_latency_overhead.
This allows the user to tune to the actual device.
*/
    static i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = freq,
        .scl_wait_us = 0,
        .flags = 0,
        //.flags = {
        //    .disable_ack_check = false,
        //},
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle));

    // Store device information in a vector for later use
    DeviceInfo device;
    device.name = name;
    device.address = address;
    device.freq = freq;
    device.read_latency = read_latency;
    device.scl_period = 1000000.0 / freq;  // in usec
    device.handle = dev_handle;
    devices_.push_back(device);
    return ESP_OK;
}

DeviceInfo AppI2cMaster::get_device_by_name(const std::string& name) {
    auto it = std::find_if(devices_.begin(), devices_.end(), [&name](const DeviceInfo& d) {
        return d.name == name;
    });
    if (it != devices_.end()) {
        return *it;
    } else {
        return DeviceInfo{};
    }
}

esp_err_t AppI2cMaster::read_bytes(const std::string name, uint8_t reg, uint8_t len, uint8_t* buffer) {
    static const char* tag = "read_bytes";
    DeviceInfo dev = get_device_by_name(name);
    //ESP_LOGD(tag, "dev.address=0x%02x", dev.address);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev.handle, &reg, 1, buffer, len, -1));
    uint8_t bits_transferred = ((3 + len) * 9);  // write address, register, read address, len, 9 scl per byte
    uint32_t delay = static_cast<uint32_t>(dev.scl_period * bits_transferred) + dev.read_latency;
    //ESP_LOGD(tag, "Delaying for %lu us", delay);
    ets_delay_us(delay);
    return ESP_OK;
}
