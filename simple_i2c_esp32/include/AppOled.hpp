#pragma once

#include "driver/gpio.h"
#include "AppI2c.hpp"
//#include "esp_ssd1306.h"

#define OTAG "OLED"

class AppOled {
public:
    explicit AppOled(uint8_t address, AppI2c* i2c);
    esp_err_t init();
    bool initialized;
private:
    uint8_t address_;
    AppI2c* i2c_;
};