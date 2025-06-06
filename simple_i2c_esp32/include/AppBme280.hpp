#pragma once

#include "driver/gpio.h"
#include "bme280.h"
#include "AppI2c.hpp"

#define BTAG "BME280"

struct SensorReading {
    float temperature;
    float pressure;
    float humidity;
    float success;
};

class AppBme280 {
public:
    explicit AppBme280(uint8_t address, AppI2c* i2c);
    void init();
    bool initialized;
    SensorReading read();
private:
    struct bme280_dev dev_;
    uint8_t address_;
    AppI2c* i2c_;
};