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
        reg_addr - <val> the register address being accessed.
        read_data - <ref> this is where the read data is returned.
    The user is required to size the read_data vector to the number of bytes being
    read. Any values in the vector are overwritten.
    */
    bme_err_t read_reg(const uint8_t reg_addr, std::vector<uint8_t>& read_data);

    /*
    write_reg
    Use this to write a bme register. Returns bme_err_t.
        reg_addr - <val> the register address being accessed.
        write_data - <ref> this is where the write data is stored.
    The user is required to initialize the write_data vector with the bytes to be
    written.
    */
    bme_err_t write_reg(const uint8_t reg_addr, const std::vector<uint8_t>& write_data);

    /*
    get_sensor_mode
    set_sensor_mode
    These methods get/set the power mode of the sensor.
        sensor_mode (get) - <ref> variable where power mode is returned.
        sensor_mode (set) - <val> variable used to set power mode.

    sensor_mode | CONSTANTS
    ------------|--------------------------------
       0        | BME280_POWERMODE_SLEEP
       1        | BME280_POWERMODE_FORCED
       3        | BME280_POWERMODE_NORMAL
    */
    bme_err_t get_sensor_mode(uint8_t &sensor_mode);
    bme_err_t set_sensor_mode(uint8_t sensor_mode);

    /*
    get_sensor_settings
    set_sensor_settings
    These methods get/set the oversampling, filter, and standby duration
    (normal mode) settings in the sensor.
      desired_settings (set) - <val> used to select which settings are being set.
    The constants used for selecting desired settings are shown below. User can do
    logical-OR of these constants to configure multiple settings.
    CONSTANTS            | Functionality
    ---------------------|---------------------------------------
    BME280_SEL_OSR_PRESS | To set pressure oversampling
    BME280_SEL_OSR_TEMP  | To set temperature oversampling
    BME280_SEL_OSR_HUM   | To set humidity oversampling
    BME280_SEL_FILTER    | To set filter
    BME280_SEL_STANDBY   | To set standby duration
    */
    bme_err_t get_sensor_settings();
    bme_err_t set_sensor_settings(uint8_t desired_settings);

    /*
    get_sensor_data
    This method reads the pressure, temperature and humidity data from the
    sensor, compensates the data, and stores it in the bme280_data structure
    instance passed in by the user.
      sensor_comp - <val> variable which selects which data to be calculated.
      comp_data - <ref> structure instance of bme280_data
    sensor_comp | CONSTANTS
    ------------|---------------------------
        1       | BME280_PRESS
        2       | BME280_TEMP
        4       | BME280_HUM
        7       | BME280_ALL
    */
    bme_err_t get_sensor_data(uint8_t sensor_comp, struct bme280_data& comp_data);

    /*
    cal_meas_delay
    This method is used to calculate the maximum delay (in us) required for the
    enabled measurement(s) to complete. The delay depends on the enabled measurements
    and their oversampling configuration.
      max_delay - <ref> delay required (in us)
    */
    bme_err_t cal_meas_delay(uint32_t& max_delay);

private:
    uint8_t address_;
    uint32_t speed_;
    uint32_t read_latency_;
    i2c_master_bus_handle_t i2c_bus_handle_;
    i2c_master_dev_handle_t sensor_handle_;
    std::vector<uint8_t> data_;
    float scl_period_;
    bme280_calibration_data_t calib_data_;
    bme280_settings_t settings_;

    gpio_num_t trigger_gpio_;
    AppGpioOut trigger_;

    uint16_t concat_bytes(uint8_t b1, uint8_t b0);
    bme_err_t soft_reset();
    bme_err_t get_calib_data();
    bme_err_t put_device_to_sleep();
    bme_err_t write_power_mode(uint8_t sensor_mode);
    bme_err_t reload_device_settings();
    bme_err_t set_osr_settings(uint8_t desired_settings);
    bme_err_t set_osr_humidity_settings();
    bme_err_t set_osr_press_temp_settings(uint8_t desired_settings);
    bme_err_t set_filter_standby_settings(uint8_t desired_settings);
    bme_err_t compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data& uncomp_data, struct bme280_data& comp_data);
    double compensate_temperature(const struct bme280_uncomp_data& uncomp_data);
    double compensate_pressure(const struct bme280_uncomp_data& uncomp_data);
    double compensate_humidity(const struct bme280_uncomp_data& uncomp_data);
    void parse_temp_press_calib_data();
    void parse_humidity_calib_data();
    void parse_device_settings(std::vector<uint8_t>& reg_data);
    void parse_sensor_data(std::vector<uint8_t>& reg_data, struct bme280_uncomp_data& uncomp_data);
    void fill_filter_settings(uint8_t& reg_data);
    void fill_standby_settings(uint8_t& reg_data);
    void fill_osr_press_settings(uint8_t& reg_data);
    void fill_osr_temp_settings(uint8_t& reg_data);
    bool are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);

};