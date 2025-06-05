#pragma once

#include <cstring>
#include "driver/i2c.h"
#include "bme280.h"

#define MTAG "MULTISENSOR"
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define BME280_ADDRESS 0x77

namespace app {
    struct SensorReading {
        float pressure;
        float temperature;
        float humidity;
        bool success;
    };

    class AppMultisensor {
    private:
        struct bme280_dev dev_;
    public:
        AppMultisensor();
        bool init();
        bool bme280_begin();
        SensorReading read(void);
    };
}

