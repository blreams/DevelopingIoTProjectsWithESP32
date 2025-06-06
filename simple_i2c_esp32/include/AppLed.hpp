#pragma once

#include "driver/gpio.h"

class AppLed {
public:
    explicit AppLed(gpio_num_t pin);
    esp_err_t init();
    void set(bool val);
    bool get();
private:
    gpio_num_t pin_;
    bool state_ = false;
};
