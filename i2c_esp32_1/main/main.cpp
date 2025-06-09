#include <iostream>
#include <stdio.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"

#include "AppI2cMaster.hpp"
#include "AppGpioOut.hpp"
#include "AppBme280.hpp"

extern "C" {

    void vBlinkTask(void* pvParameters) {
        AppGpioOut led(GPIO_NUM_2);
        ESP_ERROR_CHECK(led.init());
        while (true) {
            led.set(!led.get());
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    };

    void vGetTempTask(void* pvParameters) {
        AppI2cMaster i2c;
        const std::string bme_name = "bme280";
        AppBme280 bme280(bme_name);
        ESP_ERROR_CHECK(i2c.init());
        bme280_config_t bme_config = {
            .address = 0x77,
            .freq = I2C_MAX_FREQ_HZ,
            .read_latency = 0,
            .i2c = &i2c
        };
        ESP_ERROR_CHECK(bme280.init(bme_config));

        vTaskSuspend(nullptr);
    };

    void app_main() {
        printf("ESP-IDF version: %s\n", esp_get_idf_version());

        // create a task to blink the LED
        xTaskCreate(vBlinkTask, "blink_task", 4096, nullptr, 5, nullptr);
        xTaskCreate(vGetTempTask, "get_temp_task", 4096, nullptr, 5, nullptr);

        vTaskSuspend(nullptr);

    };
};
