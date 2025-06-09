#include <iostream>
#include <stdio.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_idf_version.h"

#include "AppI2cMaster.hpp"
#include "AppGpioOut.hpp"
#include "AppBme280.hpp"

static const char *TAG = "i2c-master";

extern "C" void app_main()
{
    printf("ESP-IDF version: %s\n", esp_get_idf_version());

    AppGpioOut led(GPIO_NUM_2);
    AppI2cMaster i2c;
    const std::string bme_name = "bme280";
    AppBme280 bme280(bme_name);


    // initialize objects
    ESP_ERROR_CHECK(led.init());
    ESP_ERROR_CHECK(i2c.init());

    bme280_config_t bme_config = {
        .address = 0x77,
        .freq = I2C_MAX_FREQ_HZ,
        .read_latency = 0,
        .i2c = &i2c
    };
    ESP_ERROR_CHECK(bme280.init(bme_config));


    //i2c.scan_bus();

    /*
    ESP_ERROR_CHECK(i2c.add_device("bme280", 0x77, I2C_MAX_FREQ_HZ));
    DeviceInfo dev = i2c.get_device_by_name("bme280");

    uint8_t reg = 0xd0;
    uint8_t read_buffer = 0;

    led.set(true);
    ESP_ERROR_CHECK(i2c.read_bytes("bme280", reg, 1, &read_buffer));
    led.set(false);
    ESP_LOGD(TAG, "Register %x read_buffer value is %x", 0xd0, read_buffer);
    */

    vTaskSuspend(NULL);

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
