#include "AppButton.hpp"

namespace {
    void gpio_isr_handler(void* arg);
}

AppButton::AppButton(std::function<void(bool)> h) : state(false), pressedHandler(h) {};

void AppButton::init() {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << GPIO_NUM_33);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_NEGEDGE;  // Falling edge button press
    gpio_config(&config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_33, gpio_isr_handler, this);
}

void AppButton::toggle() {
    state = !state;
    pressedHandler(state);
}

namespace {
    void IRAM_ATTR gpio_isr_handler(void* arg) {
        static volatile TickType_t next = 0;
        TickType_t now = xTaskGetTickCountFromISR();
        if (now > next) {
            auto btn = reinterpret_cast<AppButton *>(arg);
            btn->toggle();
            next = now + 500 / portTICK_PERIOD_MS;
        }
    }
}
