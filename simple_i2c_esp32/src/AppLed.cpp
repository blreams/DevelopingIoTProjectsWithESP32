#include "AppLed.hpp"

AppLed::AppLed(gpio_num_t pin) : pin_(pin) {}

esp_err_t AppLed::init() {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << pin_);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
    return ESP_OK;
}

void AppLed::set(bool val) {
    state_ = val;
    gpio_set_level (pin_, static_cast<uint32_t>(state_));
}

bool AppLed::get() {
    return state_;
}
