#pragma once

#include "driver/i2c.h"
#include "bme280.h"

#define ITAG "I2C"
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

class AppI2c {
public:
    explicit AppI2c(gpio_num_t scl, gpio_num_t sda);
    void init();
    bool initialized;
    static int8_t user_i2c_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr);
    static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr);
    static void user_delay_us(uint32_t period, void* intf_ptr);
    void scan_i2c_bus();
private:
    gpio_num_t scl_;
    gpio_num_t sda_;
};
