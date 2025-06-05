#include "AppMultisensor.h"
#include "esp_log.h"

void i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}
// custom i2c read function
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

static void user_delay_us(uint32_t period, void* intf_ptr) {
    ets_delay_us(period);
}

bool app::AppMultisensor::bme280_begin() {
    static uint8_t addr = BME280_ADDRESS;
    dev_.intf = BME280_I2C_INTF;
    dev_.intf_ptr = &addr;
    dev_.read = user_i2c_read;
    dev_.write = user_i2c_write;
    dev_.delay_us = user_delay_us;

    int8_t rslt = bme280_init(&dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(MTAG, "Failed bme280_init (%d)", rslt);
        return false;
    }

    bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(MTAG, "Failed bme280_set_sensor_settings (%d)", rslt);
        return false;
    }

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev_);
    if (rslt != BME280_OK) {
        ESP_LOGE(MTAG, "Failed bme280_set_sensor_mode (%d)", rslt);
        return false;
    }

    ESP_LOGI(MTAG, "bme280_begin was successful.");
    return true;
}

app::AppMultisensor::AppMultisensor() {
    // empty constructor
}

bool app::AppMultisensor::init() {
    bool init_success = true;
    i2c_master_init();
    init_success &= bme280_begin();
    return init_success;
}

app::SensorReading app::AppMultisensor::read() {
    bme280_data comp_data;
    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_);
    SensorReading reading = {0.0f, 0.0f, 0.0f, false};

    if (rslt != BME280_OK) {
        ESP_LOGE(MTAG, "Failed bme280_get_sensor_data (%d)", rslt);
        reading.success = false;
        return reading;
    }

    reading.temperature = comp_data.temperature;
    reading.pressure = comp_data.pressure;
    reading.humidity = comp_data.humidity;
    reading.success = true;
    return reading;
}