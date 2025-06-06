#include "esp_log.h"
#include "AppBme280.hpp"

AppBme280::AppBme280(uint8_t address, AppI2c* i2c) : address_(address), i2c_(i2c) {};

float c_to_f(float ctemp) { return ((ctemp * (9.0 / 5.0)) + 32.0); }
float p_to_i(float ppres) { return (ppres * 0.0002952998751); }

esp_err_t AppBme280::init() {
    initialized = false;
    //static uint8_t addr = 0x77;
    dev_.intf = BME280_I2C_INTF;
    dev_.intf_ptr = &address_;
    dev_.read = i2c_->user_i2c_read;
    dev_.write = i2c_->user_i2c_write;
    dev_.delay_us = i2c_->user_delay_us;

    int8_t rslt = bme280_init(&dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(BTAG, "Failed bme280_init (%d)", rslt);
        return ESP_FAIL;
    }

    bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(BTAG, "Failed bme280_set_sensor_settings (%d)", rslt);
        return ESP_FAIL;
    }

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(BTAG, "Failed bme280_set_sensor_mode (%d)", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(BTAG, "begin() was successful.");
    initialized = true;
    return ESP_OK;
}


SensorReading AppBme280::read() {
    bme280_data comp_data;
    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_);
    SensorReading reading = {0.0f, 0.0f, 0.0f, false};

    if (rslt != BME280_OK) {
        ESP_LOGE(BTAG, "Failed bme280_get_sensor_data (%d)", rslt);
        reading.success = false;
        return reading;
    }

    reading.temperature = c_to_f(comp_data.temperature);
    reading.pressure = p_to_i(comp_data.pressure);
    reading.humidity = comp_data.humidity;
    reading.success = true;
    return reading;
}
