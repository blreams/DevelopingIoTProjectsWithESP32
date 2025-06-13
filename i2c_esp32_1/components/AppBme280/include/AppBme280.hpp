#pragma once

#include <string>
#include <vector>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bme_defs.hpp"
#include "AppGpioOut.hpp"

class AppBme280 {
public:
    explicit AppBme280(
        uint8_t i2c_address,
        uint32_t i2c_speed,
        uint32_t read_latency=100,
        gpio_num_t trigger_gpio=GPIO_NUM_32
    );

    /*
    init
    Assumes the i2c bus has been created because it requires the bus handle.
    Will call i2c_master_bus_add_device. The private member sensor_handle_ is
    assigned from this call.
        i2c_bus_handle - <ref> handle from calling i2c_master_bus_add_device.
    */
    bme_err_t init(i2c_master_bus_handle_t& i2c_bus_handle);

    /*
    start
    The following steps take place:
        1. Check the ID of the sensor (verifies that we can communicate over i2c).
        2. Soft reset the sensor.
        3. Insure NVM is copied.
    */
    bme_err_t start();
    /*
    read_reg
    Use this to read a bme register. Returns bme_err_t.
        reg_addr - <value> the register address being accessed.
        read_data - <ref> this is where the read data is returned.
    The user is required to size the read_data vector to the number of bytes being
    read. Any values in the vector are overwritten.
    */
    bme_err_t read_reg(const uint8_t reg_addr, std::vector<uint8_t>& read_data);

    /*
    write_reg
    Use this to write a bme register. Returns bme_err_t.
        reg_addr - <value> the register address being accessed.
        write_data - <ref> this is where the write data is stored.
    The user is required to initialize the write_data vector with the bytes to be
    written.
    */
    bme_err_t write_reg(const uint8_t reg_addr, const std::vector<uint8_t>& write_data);

    bme_err_t soft_reset();
    bme_err_t get_calib_data();
    void parse_temp_press_calib_data();
    void parse_humidity_calib_data();
    bme_err_t write_power_mode(uint8_t sensor_mode);

private:
    uint8_t address_;
    uint32_t speed_;
    uint32_t read_latency_;
    i2c_master_bus_handle_t i2c_bus_handle_;
    i2c_master_dev_handle_t sensor_handle_;
    std::vector<uint8_t> data_;
    float scl_period_;
    bme280_calibration_data_t calib_data_;  // TODO

    esp_err_t i2c_add_device(uint8_t address, uint32_t scl_hz);
    uint16_t concat_bytes(uint8_t b1, uint8_t b0);

    gpio_num_t trigger_gpio_;
    AppGpioOut trigger_;
};