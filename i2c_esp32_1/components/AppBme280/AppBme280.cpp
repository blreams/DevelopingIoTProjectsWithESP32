#include "AppBme280.hpp"

AppBme280::AppBme280(
    uint8_t address,
    uint32_t speed,
    uint32_t read_latency,
    gpio_num_t trigger_gpio
) : 
    address_(address),
    speed_(speed),
    read_latency_(read_latency),
    trigger_gpio_(trigger_gpio), 
    trigger_(trigger_gpio)
{
    data_.reserve(32);
}

bme_err_t AppBme280::init(i2c_master_bus_handle_t& i2c_bus_handle) {
    i2c_device_config_t sensor_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address_,
        .scl_speed_hz = speed_,
        .scl_wait_us = 0,
        .flags = 0,
    };
    scl_period_ = 1000000.0 / speed_;  // in usec
    return i2c_master_bus_add_device(i2c_bus_handle, &sensor_config, &sensor_handle_);
}

bme_err_t AppBme280::start() {
    const char* tag = "AppBme280::start";
    // Configure trigger gpio
    trigger_.init();
    trigger_.set(false);

    // try reading the ID register
    data_.resize(1);
    read_reg(BME280_REG_CHIP_ID, data_);

    if (data_[0] != BME280_CHIP_ID) {
        return BME280_ERR_COMM_FAIL;
    }

    ESP_ERROR_CHECK(soft_reset());
    ESP_ERROR_CHECK(get_calib_data());

    ESP_LOGD(tag, "init() complete");
    return BME280_OK;
}

bme_err_t AppBme280::soft_reset() {
    static const char* tag = "AppBme280::soft_reset";
    // write to the RESET register
    std::vector<uint8_t> write_data = {BME280_SOFT_RESET_COMMAND};
    ESP_LOGD(tag, "writing to reset reg");
    ESP_ERROR_CHECK(write_reg(BME280_REG_RESET, write_data));
    ESP_LOGD(tag, "write to reset reg complete");

    // wait for NVM copy to complete
    bme_err_t rslt = 0;
    uint8_t try_run = 5;
    uint8_t status_reg = 0;
    data_.resize(1);
    trigger_.set(true);
    do {
        // data sheet Table 1, startup time is 2 ms
        //vTaskDelay(pdMS_TO_TICKS(BME280_STARTUP_DELAY));
        ets_delay_us(BME280_STARTUP_DELAY);
        rslt = read_reg(BME280_REG_STATUS, data_);
        ESP_LOGD(tag, "status_reg=%02x", data_[0]);
        if (rslt == BME280_OK) {
            status_reg = data_[0];
        }
    } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));
    trigger_.set(false);

    if (status_reg & BME280_STATUS_IM_UPDATE) {
        rslt = BME280_ERR_NVM_COPY_FAILED;
    }
    return rslt;
}

bme_err_t AppBme280::get_calib_data() {
    const char* tag = "AppBme280::get_calib_data";
    bme_err_t rslt;
    data_.resize(BME280_LEN_TEMP_PRESS_CALIB_DATA);
    rslt = read_reg(BME280_REG_TEMP_PRESS_CALIB_DATA, data_);
    parse_temp_press_calib_data();

    data_.resize(BME280_LEN_HUMIDITY_CALIB_DATA);
    rslt = read_reg(BME280_REG_HUMIDITY_CALIB_DATA, data_);
    parse_humidity_calib_data();

    return rslt;
}

void AppBme280::parse_temp_press_calib_data() {
    calib_data_.dig_t1 = concat_bytes(data_[1], data_[0]);                          // 89:88
    calib_data_.dig_t2 = static_cast<int16_t>(concat_bytes(data_[3], data_[2]));    // 8b:8a
    calib_data_.dig_t3 = static_cast<int16_t>(concat_bytes(data_[5], data_[4]));    // 8d:8c
    calib_data_.dig_p1 = concat_bytes(data_[7], data_[6]);                          // 8f:8e
    calib_data_.dig_p2 = static_cast<int16_t>(concat_bytes(data_[9], data_[8]));    // 91:90
    calib_data_.dig_p3 = static_cast<int16_t>(concat_bytes(data_[11], data_[10]));  // 93:92
    calib_data_.dig_p4 = static_cast<int16_t>(concat_bytes(data_[13], data_[12]));  // 95:94
    calib_data_.dig_p5 = static_cast<int16_t>(concat_bytes(data_[15], data_[14]));  // 97:96
    calib_data_.dig_p6 = static_cast<int16_t>(concat_bytes(data_[17], data_[16]));  // 99:98
    calib_data_.dig_p7 = static_cast<int16_t>(concat_bytes(data_[19], data_[18]));  // 9b:9a
    calib_data_.dig_p8 = static_cast<int16_t>(concat_bytes(data_[21], data_[20]));  // 9d:9c
    calib_data_.dig_p9 = static_cast<int16_t>(concat_bytes(data_[23], data_[22]));  // 9f:9e
    calib_data_.dig_h1 = data_[25];                                                 // a1
}

void AppBme280::parse_humidity_calib_data() {
    calib_data_.dig_h2 = static_cast<int16_t>(concat_bytes(data_[1], data_[0]));    // e2:e1
    calib_data_.dig_h3 = data_[2];                                                  // e3
    int16_t msb = static_cast<int16_t>(static_cast<int8_t>(data_[3]) * 16);
    int16_t lsb = static_cast<int16_t>(data_[4] & 0x0f);
    calib_data_.dig_h4 = msb | lsb;                                                 // e4:e5[3:0]
    msb = static_cast<int16_t>(static_cast<int8_t>(data_[5]) * 16);
    lsb = static_cast<int16_t>(data_[4] >> 4);
    calib_data_.dig_h5 = msb | lsb;                                                 // e6:e5[7:4]
    calib_data_.dig_h6 = static_cast<int8_t>(data_[6]);                             // e7
}

uint16_t AppBme280::concat_bytes(uint8_t b1, uint8_t b0) {
    return ((b1 << 8) | b0);
}

bme_err_t AppBme280::read_reg(const uint8_t reg_addr, std::vector<uint8_t>& read_data) {
    const char* tag = "AppBme280::read_read";
    bme_err_t rslt;
    uint32_t bits_to_transfer = ((3 + read_data.size()) * 9);
    uint32_t delay = static_cast<uint32_t>(scl_period_ * bits_to_transfer) + read_latency_;
    ESP_LOGD(tag, "read_reg delay=%lu", delay);
    trigger_.set(true);
    rslt = i2c_master_transmit_receive(
        sensor_handle_,
        &reg_addr,
        1,
        read_data.data(),
        read_data.size(),
        -1
    );
    if (rslt == ESP_OK) {
        ets_delay_us(delay);
    }
    trigger_.set(false);
    ESP_LOGD(tag, "Register Read Results:");
    for (size_t i=0; i < data_.size(); i++) {
        ESP_LOGD(tag, "reg[0x%02x]=0x%02x", reg_addr + i, data_[i]);
    }
    ESP_LOGD(tag, "read_reg complete status=(%d)", rslt);
    return rslt;
}

bme_err_t AppBme280::write_reg(const uint8_t reg_addr, const std::vector<uint8_t>& write_data) {
    data_.resize(2 * write_data.size());
    for (size_t i=0; i < write_data.size(); i++) {
        data_[i] = reg_addr + i;
        data_[i+1] = write_data[i];
    }
    return i2c_master_transmit(
        sensor_handle_,
        data_.data(),
        data_.size(),
        -1
    );
}
