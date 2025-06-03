#include "BME280Wrapper.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "BME280"
#define I2C_MASTER_NUM         I2C_NUM_0
#define BME280_ADDRESS         0x77

// Your custom I2C read function
static int8_t user_i2c_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

// Your custom I2C write function
static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

// Your custom delay function (microseconds)
static void user_delay_us(uint32_t period, void* intf_ptr) {
    ets_delay_us(period);
}

BME280Wrapper::BME280Wrapper() {
    // Empty constructor
}

bool BME280Wrapper::begin() {
    static uint8_t addr = BME280_ADDRESS;
    dev.intf = BME280_I2C_INTF;
    dev.intf_ptr = &addr;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    int8_t rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280 (%d)", rslt);
        return false;
    }

    bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Failed to set sensor settings (%d)", rslt);
        return false;
    }

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Failed to set sensor mode (%d)", rslt);
        return false;
    }

    ESP_LOGI(TAG, "BME280 initialized successfully");
    return true;
}

bool BME280Wrapper::readSensorData(float& temperature, float& pressure, float& humidity) {
    bme280_data comp_data;
    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data (%d)", rslt);
        return false;
    }

    temperature = comp_data.temperature;
    pressure = comp_data.pressure / 100.0f;
    humidity = comp_data.humidity;

    return true;
}
