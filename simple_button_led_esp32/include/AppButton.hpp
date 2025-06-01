#pragma once
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class AppButton {
public:
    explicit AppButton(gpio_num_t pin);
    void init();
    bool getState() const;
    void handleInterrupt();

private:
    gpio_num_t pin_;
    volatile bool toggled_state_;
    volatile TickType_t last_isr_time_;
};
