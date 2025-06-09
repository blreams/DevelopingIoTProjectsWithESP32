#include <iostream>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "AppI2cMaster.hpp"

#include "esp_idf_version.h"

#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         21
#define SCL_PIN         22
#define I2C_FREQ_HZ     400000
#define SLAVE_ADDR      0x3C  // Example address, e.g. OLED

static const char *TAG = "i2c-master";

AppI2cMaster i2c(GPIO_NUM_21, GPIO_NUM_22, 400000);

extern "C" void app_main()
{
    printf("ESP-IDF version: %s\n", esp_get_idf_version());
    ESP_ERROR_CHECK(i2c.init());
    ESP_ERROR_CHECK(i2c.add_device("bme280", 0x77, 100000));
    uint8_t buffer[8];
    ESP_ERROR_CHECK(i2c.read_bytes("bme280", 0xd0, 1, buffer));
    ESP_LOGD(TAG, "Register %x value is %x", 0xd0, buffer[0]);
    ESP_LOGD(TAG, "Register %x value is %x", 0xd0, buffer[7]);
    /*
    // 3. Transmit some data
    uint8_t bytes[] = {0x00, 0xA5};  // Example: write two bytes
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, bytes, sizeof(bytes), -1));
    ESP_LOGI(TAG, "I2C write complete");

    // 4. Optionally receive
    uint8_t rx = 0;
    uint8_t reg = 0x01;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, &rx, 1, -1));
    ESP_LOGI(TAG, "I2C read: 0x%02X", rx);
    */
}
