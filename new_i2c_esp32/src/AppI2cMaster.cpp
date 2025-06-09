#include <stdexcept>
#include <algorithm>
#include "AppI2cMaster.hpp"
#include "esp_log.h"


AppI2cMaster::AppI2cMaster(gpio_num_t scl, gpio_num_t sda, uint32_t freq_hz) :
    scl_(scl),
    sda_(sda),
    freq_hz_(freq_hz)
{};

esp_err_t AppI2cMaster::init() {
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = sda_,
        .scl_io_num = scl_,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 10,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle_));
    return ESP_OK;
}

esp_err_t AppI2cMaster::add_device(std::string name, uint8_t address, uint32_t freq) {
    // attach device on the bus
    static i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = freq,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle));

    // Store device information in a vector for later use
    DeviceInfo device;
    device.name = name;
    device.address = address;
    device.freq = freq;
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
    char tag[] = "read_bytes";
    i2c_master_dev_handle_t dev_handle = get_device_by_name(name).handle;
    i2c_device_config_t dev_cfg;
    //ESP_ERROR_CHECK(i2c_master_get_device_config(dev_handle, &dev_cfg));
    //ESP_LOGD(tag, "i2c_address=0x%02x", dev_cfg.device_address);
    ESP_ERROR_CHECK(
        i2c_master_transmit_receive(
            dev_handle,
            &reg,
            1,
            buffer,
            len,
            -1
        )
    );
    return ESP_OK;
}