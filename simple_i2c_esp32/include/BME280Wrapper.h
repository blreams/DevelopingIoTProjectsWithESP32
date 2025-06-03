#pragma once

extern "C" {
    #include "bme280.h"
}

#include <stdint.h>

class BME280Wrapper {
public:
    BME280Wrapper();
    bool begin();  // initializes I2C and the sensor
    bool readSensorData(float &temperature, float &pressure, float &humidity);

private:
    struct bme280_dev dev;
};
