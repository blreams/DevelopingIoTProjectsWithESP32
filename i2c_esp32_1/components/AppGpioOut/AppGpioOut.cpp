#include "AppGpioOut.hpp"

AppGpioOut::AppGpioOut(gpio_num_t gpio_num) : gpio_num_(gpio_num) {}

esp_err_t AppGpioOut::init() {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << gpio_num_);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config);
}

void AppGpioOut::set(bool val) {
    state_ = val;
    gpio_set_level(gpio_num_, static_cast<uint32_t>(state_));
}

bool AppGpioOut::get() {
    return state_;
}
