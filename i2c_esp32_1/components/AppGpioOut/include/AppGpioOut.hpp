#pragma once

#include "driver/gpio.h"

class AppGpioOut {
public:
    explicit AppGpioOut(gpio_num_t gpio_num);
    esp_err_t init();
    void set(bool val);
    bool get();
private:
    gpio_num_t gpio_num_;
    bool state_ = false;
};