#include "AppButton.hpp"

namespace {
    void IRAM_ATTR gpio_isr_handler(void* arg) {
        auto* btn = static_cast<AppButton*>(arg);
        btn->handleInterrupt();
    }
}

AppButton::AppButton(gpio_num_t pin) :
    pin_(pin),
    toggled_state_(false),
    last_isr_time_(0)
{}

void AppButton::init() {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << pin_);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_NEGEDGE;  // Falling edge button press
    gpio_config(&config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_, gpio_isr_handler, this);
}

void AppButton::handleInterrupt() {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - last_isr_time_) > pdMS_TO_TICKS(50)) {  // 50 ms debounce}
        toggled_state_ = !toggled_state_;
        last_isr_time_ = now;
    }
}

bool AppButton::getState() const {
    return toggled_state_;
}
