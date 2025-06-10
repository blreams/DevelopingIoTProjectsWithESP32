#include "AppBme280.hpp"

AppBme280::AppBme280(const std::string name, gpio_num_t trigger_gpio) : 
    name_(name), 
    trigger_gpio_(trigger_gpio), 
    trigger_(trigger_gpio) {}

bme_err_t AppBme280::init(bme280_config_t config) {
    // Configure trigger gpio
    trigger_.init();
    trigger_.set(false);

    static const char* tag = "AppBme280::init";
    config_ = config;
    ESP_ERROR_CHECK(config_.i2c->add_device(name_, config.address, config.freq));

    // try reading the ID register
    ESP_ERROR_CHECK(read_reg(BME280_REG_CHIP_ID, 1));

    if (reg_data_[0] != BME280_CHIP_ID) {
        return BME280_ERR_COMM_FAIL;
    }
    dev_.chip_id = reg_data_[0];

    ESP_ERROR_CHECK(soft_reset());

    ESP_LOGD(tag, "init() complete");
    return BME280_OK;
}

bme_err_t AppBme280::soft_reset() {
    static const char* tag = "AppBme280::soft_reset";
    // write to the RESET register
    uint8_t reg_addr = BME280_REG_RESET;
    std::vector<uint8_t> data = {BME280_SOFT_RESET_COMMAND};
    ESP_LOGD(tag, "writing to reset reg");
    ESP_ERROR_CHECK(write_reg(reg_addr, 1, data));
    ESP_LOGD(tag, "write to reset reg complete");

    // wait for NVM copy to complete
    bme_err_t rslt = 0;
    uint8_t try_run = 5;
    uint8_t status_reg = 0;
    trigger_.set(true);
    do {
        // data sheet Table 1, startup time is 2 ms
        //vTaskDelay(pdMS_TO_TICKS(BME280_STARTUP_DELAY));
        ets_delay_us(BME280_STARTUP_DELAY);
        rslt = config_.i2c->read_bytes(name_, BME280_REG_STATUS, 1, reg_data_);
        ESP_LOGD(tag, "status_reg=%02x", reg_data_[0]);
        if (rslt == BME280_OK) {
            status_reg = reg_data_[0];
        }
    } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));
    trigger_.set(false);

    if (status_reg & BME280_STATUS_IM_UPDATE) {
        rslt = BME280_ERR_NVM_COPY_FAILED;
    }
    return rslt;
}

bme_err_t AppBme280::read_reg(const uint8_t reg_addr, uint8_t len) {
    reg_data_.resize(len);
    return config_.i2c->read_bytes(name_, reg_addr, len, reg_data_);
}

bme_err_t AppBme280::write_reg(const uint8_t reg_addr, uint8_t len, std::vector<uint8_t>& write_data) {
    if (len == 1) {
        uint8_t data = write_data[0];
        return config_.i2c->write_byte(name_, reg_addr, data);
    }
    return BME280_ERR_COMM_FAIL;
}
