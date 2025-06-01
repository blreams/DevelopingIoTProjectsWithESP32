#pragma once
#include "driver/gpio.h"

class AppLed {
public:
    explicit AppLed(gpio_num_t pin);
    void init();
    void set(bool val);

private:
    gpio_num_t pin_;
};