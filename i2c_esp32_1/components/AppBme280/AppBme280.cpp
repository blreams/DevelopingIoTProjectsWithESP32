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
    //calib_data_({0}),
    //settings_({0}),

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
    bme_err_t rslt;
    data_.resize(BME280_LEN_TEMP_PRESS_CALIB_DATA);
    rslt = read_reg(BME280_REG_TEMP_PRESS_CALIB_DATA, data_);
    parse_temp_press_calib_data();

    data_.resize(BME280_LEN_HUMIDITY_CALIB_DATA);
    rslt = read_reg(BME280_REG_HUMIDITY_CALIB_DATA, data_);
    parse_humidity_calib_data();

    return rslt;
}

void parse_sensor_data(std::vector<uint8_t>& reg_data, struct bme280_uncomp_data& uncomp_data) {
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    // store the parsed register values for pressure
    data_msb = static_cast<uint32_t>(reg_data[0]) << BME280_12_BIT_SHIFT;
    data_lsb = static_cast<uint32_t>(reg_data[1]) << BME280_4_BIT_SHIFT;
    data_xlsb = static_cast<uint32_t>(reg_data[2]) << BME280_4_BIT_SHIFT;
    uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

    // store the parsed register values for temperature
    data_msb = static_cast<uint32_t>(reg_data[3]) << BME280_12_BIT_SHIFT;
    data_lsb = static_cast<uint32_t>(reg_data[4]) << BME280_4_BIT_SHIFT;
    data_xlsb = static_cast<uint32_t>(reg_data[5]) << BME280_4_BIT_SHIFT;
    uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

    // store the parsed register values for humidity
    data_msb = static_cast<uint32_t>(reg_data[6]) << BME280_8_BIT_SHIFT;
    data_lsb = static_cast<uint32_t>(reg_data[7]);
    uncomp_data.temperature = data_msb | data_lsb;
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

bme_err_t AppBme280::get_sensor_mode(uint8_t &sensor_mode) {
    bme_err_t rslt = {BME280_OK};

    data_.resize(1);
    rslt = read_reg(BME280_REG_PWR_CTRL, data_);
    sensor_mode = data_[0];
    return rslt;
}

bme_err_t AppBme280::set_sensor_mode(uint8_t sensor_mode) {
    bme_err_t rslt = {BME280_OK};
    uint8_t last_sensor_mode;

    data_.resize(1);
    rslt = get_sensor_mode(last_sensor_mode);

    // if sensor is not in sleep mode, put it to sleep
    if ((rslt == BME280_OK) && (last_sensor_mode != BME280_POWERMODE_SLEEP)) {
        rslt = put_device_to_sleep();
    }

    // set power mode
    if (rslt == BME280_OK) {
        rslt = write_power_mode(sensor_mode);
    }
    return rslt;
}

bme_err_t AppBme280::get_sensor_settings() {
    bme_err_t rslt = BME280_OK;

    data_.resize(4);
    rslt = read_reg(BME280_REG_CTRL_HUM, data_);

    if (rslt == BME280_OK) {
        parse_device_settings(data_);
    }
    return rslt;
}

bme_err_t AppBme280::set_sensor_settings(uint8_t desired_settings) {
    bme_err_t rslt = BME280_OK;
    uint8_t sensor_mode;

    rslt = get_sensor_mode(sensor_mode);

    if ((rslt == BME280_OK) && (sensor_mode != BME280_POWERMODE_SLEEP)) {
        rslt = put_device_to_sleep();
    }

    if (rslt == BME280_OK) {
        // check if user want to change oversampling
        if (are_settings_changed(BME280_OVERSAMPLING_SETTINGS, desired_settings)) {
            rslt = set_osr_settings(desired_settings);
        }

        // check if user want to change filter and/or standby
        if ((rslt == BME280_OK) && are_settings_changed(BME280_FILTER_STANDBY_SETTINGS, desired_settings)) {
            rslt = set_filter_standby_settings(desired_settings);
        }
    }
    return rslt;
}

void AppBme280::parse_device_settings(std::vector<uint8_t>& reg_data) {
    settings_.osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
    settings_.osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
    settings_.osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
    settings_.filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
    settings_.standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

void AppBme280::fill_filter_settings(uint8_t& reg_data) {
    reg_data = BME280_SET_BITS(reg_data, BME280_FILTER, settings_.filter);
}

void AppBme280::fill_standby_settings(uint8_t& reg_data) {
    reg_data = BME280_SET_BITS(reg_data, BME280_STANDBY, settings_.standby_time);
}

void AppBme280::fill_osr_press_settings(uint8_t& reg_data) {
    reg_data = BME280_SET_BITS(reg_data, BME280_CTRL_PRESS, settings_.osr_p);
}

void AppBme280::fill_osr_temp_settings(uint8_t& reg_data) {
    reg_data = BME280_SET_BITS(reg_data, BME280_CTRL_TEMP, settings_.osr_t);
}

bme_err_t AppBme280::set_filter_standby_settings(uint8_t desired_settings) {
    bme_err_t rslt = BME280_OK;
    data_.resize(1);
    rslt = read_reg(BME280_REG_CONFIG, data_);

    if (rslt == BME280_OK) {
        if (desired_settings & BME280_SEL_FILTER) {
            fill_filter_settings(data_[0]);
        }

        if (desired_settings & BME280_SEL_STANDBY) {
            fill_standby_settings(data_[0]);
        }

        rslt = write_reg(BME280_REG_CONFIG, data_);
    }
    return rslt;
}

bme_err_t AppBme280::set_osr_settings(uint8_t desired_settings) {
    bme_err_t rslt = {BME280_WARN_INVALID_OSR_MACRO};

    if (desired_settings & BME280_SEL_OSR_HUM) {
        rslt = set_osr_humidity_settings();
    }

    if (desired_settings & (BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP)) {
        rslt = set_osr_press_temp_settings(desired_settings);
    }
    return rslt;
}

bme_err_t AppBme280::set_osr_humidity_settings() {
    bme_err_t rslt = BME280_OK;
    data_.resize(1);
    data_[0] = settings_.osr_h & BME280_CTRL_HUM_MSK;
    // write the humidity control value
    rslt = write_reg(BME280_REG_CTRL_HUM, data_);

    // humidity related changes will only be effective after writing to
    // ctrl_meas reg, read it, then write it back
    if (rslt == BME280_OK) {
        rslt = read_reg(BME280_REG_CTRL_MEAS, data_);

        if (rslt == BME280_OK) {
            rslt = write_reg(BME280_REG_CTRL_MEAS, data_);
        }
    }
    return rslt;
}

bme_err_t AppBme280::set_osr_press_temp_settings(uint8_t desired_settings) {
    bme_err_t rslt = BME280_OK;

    data_.resize(1);
    rslt = read_reg(BME280_REG_CTRL_MEAS, data_);

    if (rslt == BME280_OK) {
        if (desired_settings & BME280_SEL_OSR_PRESS) {
            fill_osr_press_settings(data_[0]);
        }

        if (desired_settings & BME280_SEL_OSR_TEMP) {
            fill_osr_temp_settings(data_[0]);
        }

        rslt = write_reg(BME280_REG_CTRL_MEAS, data_);
    }
    return rslt;
}

bme_err_t AppBme280::reload_device_settings() {
    bme_err_t rslt = {BME280_OK};

    rslt = set_osr_settings(BME280_SEL_ALL_SETTINGS);

    if (rslt == BME280_OK) {
        rslt = set_filter_standby_settings(BME280_SEL_ALL_SETTINGS);
    }
    return rslt;
}

bme_err_t AppBme280::put_device_to_sleep() {
    bme_err_t rslt = {BME280_OK};

    data_.resize(4);
    rslt = read_reg(BME280_REG_CTRL_HUM, data_);

    if (rslt == BME280_OK) {
        parse_device_settings(data_);
        rslt = soft_reset();

        if (rslt == BME280_OK) {
            rslt = reload_device_settings();
        }
    }
    return rslt;
}

bme_err_t AppBme280::write_power_mode(uint8_t sensor_mode) {
    bme_err_t rslt;

    data_.resize(1);
    rslt = read_reg(BME280_REG_PWR_CTRL, data_);
    if (rslt == BME280_OK) {
        data_[0] = BME280_SET_BITS_POS_0(data_[0], BME280_SENSOR_MODE, sensor_mode);
        rslt = write_reg(BME280_REG_PWR_CTRL, data_);
    }
    return rslt;
}

bool AppBme280::are_settings_changed(uint8_t sub_settings, uint8_t desired_settings) {
    bool settings_changed = false;

    if (sub_settings & desired_settings) {
        // user wants to modify this particular setting
        settings_changed = true;
    } else {
        settings_changed = false;
    }
    return settings_changed;
}

bme_err_t AppBme280::get_sensor_data(uint8_t sensor_comp, struct bme280_data& comp_data) {
    bme_err_t rslt = {BME280_OK};
    struct bme280_uncomp_data uncomp_data = {
        .pressure = 0,
        .temperature = 0,
        .humidity = 0,
    };

    // read temp/press data from sensor
    data_.resize(BME280_LEN_P_T_H_DATA);
    rslt = read_reg(BME280_REG_DATA, data_);

    if (rslt == BME280_OK) {
        parse_sensor_data(data_, uncomp_data);
        // compensate
        rslt = compensate_data(sensor_comp, uncomp_data, comp_data);
    }
    return rslt;
}

bme_err_t AppBme280::compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data& uncomp_data, struct bme280_data& comp_data) {
    bme_err_t rslt = {BME280_OK};

    // initialize comp_data to zero
    comp_data.temperature = 0;
    comp_data.pressure = 0;
    comp_data.humidity = 0;

    if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) {
        comp_data.temperature = compensate_temperature(uncomp_data);
    }

    if (sensor_comp & BME280_PRESS) {
        comp_data.pressure = compensate_pressure(uncomp_data);
    }

    if (sensor_comp & BME280_HUM) {
        comp_data.humidity = compensate_humidity(uncomp_data);
    }

    return rslt;
}

double AppBme280::compensate_temperature(const struct bme280_uncomp_data& uncomp_data) {
    double var1;
    double var2;
    double temperature;
    double temperature_min = {-40.0};
    double temperature_max = {85.0};

    var1 = ((static_cast<double>(uncomp_data.temperature) / 16384.0) - (static_cast<double>(calib_data_.dig_t1) / 1024.0)) * (static_cast<double>(calib_data_.dig_t2));
    var2 = ((static_cast<double>(uncomp_data.temperature) / 131072.0) - (static_cast<double>(calib_data_.dig_t1) / 8192.0));
    var2 = (var2 * var2) * (static_cast<double>(calib_data_.dig_t3));
    calib_data_.t_fine = (static_cast<int32_t>(var1 + var2));
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min) {
        temperature = temperature_min;
    } else if (temperature > temperature_max) {
        temperature = temperature_max;
    }
    return temperature;
}

double AppBme280::compensate_pressure(const struct bme280_uncomp_data& uncomp_data) {
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = (static_cast<double>(calib_data_.t_fine) / 2.0) - 64000.0;
    var2 = var1 * var1 * (static_cast<double>(calib_data_.dig_p6) / 32768.0);
    var2 = var2 + (var1 * (static_cast<double>(calib_data_.dig_p5) * 2.0));
    var2 = (var2 / 4.0) + (static_cast<double>(calib_data_.dig_p4) * 65536.0);
    var3 = ((static_cast<double>(calib_data_.dig_p3) * var1) / 524288.0);
    var1 = ((var3 + (static_cast<double>(calib_data_.dig_p2) * var1)) / 524288.0);
    var1 = (1.0 + (var1 / 32768.0)) * (static_cast<double>(calib_data_.dig_p1));

    // avoid DIV0 exception
    pressure = pressure_min;
    if (var1 > (0.0)) {
        pressure = 1048576.0 - (static_cast<double>(uncomp_data.pressure));
        pressure = (pressure - (var2 / 4096.0)) * (6250.0 / var1);
        var1 = (static_cast<double>(calib_data_.dig_p9)) * pressure * pressure / 2147483648.0;
        var2 = pressure * (static_cast<double>(calib_data_.dig_p8)) / 32768.0;
        pressure = pressure + (var1 + var2 + (static_cast<double>(calib_data_.dig_p7)) / 16.0);

        if (pressure < pressure_min) {
            pressure = pressure_min;
        } else if (pressure > pressure_max) {
            pressure = pressure_max;
        }
    }
    return pressure;
}

double AppBme280::compensate_humidity(const struct bme280_uncomp_data& uncomp_data) {
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = static_cast<double>(calib_data_.t_fine) - 76800.0;
    var2 = (static_cast<double>(calib_data_.dig_h4) * 64.0) + ((static_cast<double>(calib_data_.dig_h5) / 16384.0) * var1);
    var3 = static_cast<double>(uncomp_data.humidity) - var2;
    var4 = static_cast<double>(calib_data_.dig_h2) / 65536.0;
    var5 = (1.0 + ((static_cast<double>(calib_data_.dig_h3) / 67108864.0) * var1));
    var6 = 1.0 + ((static_cast<double>(calib_data_.dig_h6) / 67108864.0) * var1 * var5);
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - (static_cast<double>(calib_data_.dig_h1) * var6 / 524288.0));

    if (humidity > humidity_max) {
        humidity = humidity_max;
    } else if (humidity < humidity_min) {
        humidity = humidity_min;
    }
    return humidity;
}

bme_err_t AppBme280::cal_meas_delay(uint32_t& max_delay) {
    bme_err_t rslt = {BME280_OK};
    uint8_t temp_osr = {BME280_OVERSAMPLING_16X};
    uint8_t pres_osr = {BME280_OVERSAMPLING_16X};
    uint8_t hum_osr = {BME280_OVERSAMPLING_16X};

    // vector to map OSR config register value to actual OSR
    std::vector<uint8_t> osr_sett_to_act_osr = { 0, 1, 2, 4, 8, 16 };

    if (settings_.osr_t <= BME280_OVERSAMPLING_16X) {
        temp_osr = osr_sett_to_act_osr[settings_.osr_t];
    }

    if (settings_.osr_p <= BME280_OVERSAMPLING_16X) {
        pres_osr = osr_sett_to_act_osr[settings_.osr_p];
    }

    if (settings_.osr_h <= BME280_OVERSAMPLING_16X) {
        hum_osr = osr_sett_to_act_osr[settings_.osr_h];
    }

    max_delay = static_cast<uint32_t>(
        (BME280_MEAS_OFFSET + (BME280_MEAS_DUR * temp_osr)) +
        (BME280_PRES_HUM_MEAS_OFFSET + (BME280_MEAS_DUR * pres_osr)) +
        (BME280_PRES_HUM_MEAS_OFFSET + (BME280_MEAS_DUR * hum_osr))
    );

    return rslt;
}