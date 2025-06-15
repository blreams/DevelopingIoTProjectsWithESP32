#include <iostream>
#include <stdio.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"

#include "AppGpioOut.hpp"
#include "AppBme280.hpp"

#define BME280_I2C_ADDRESS 0x77
#define BME280_I2C_SPEED 400000
#define BME280_I2C_READ_LATENCY 150

SemaphoreHandle_t i2c_bus_init_done;
SemaphoreHandle_t sensor_init_done;

i2c_master_bus_handle_t i2c_bus_handle;
AppBme280 bme(BME280_I2C_ADDRESS, BME280_I2C_SPEED, BME280_I2C_READ_LATENCY);

extern "C" {
    void vTaskInitI2c(void* pvParameters) {
        // initialize i2c bus_config
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_21,
            .scl_io_num = GPIO_NUM_22,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 1,
            .flags = {
                .enable_internal_pullup = 1,
                .allow_pd = 0,
            }
        };
        // create new i2c bus
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
        xSemaphoreGive(i2c_bus_init_done);
        vTaskDelete(nullptr);
    }

    void vTaskInitSensor(void* pvParameters) {
        // initialize sensor i2c device
        i2c_master_dev_handle_t sensor_handle;
        i2c_device_config_t sensor_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BME280_I2C_ADDRESS,
            .scl_speed_hz = BME280_I2C_SPEED,
            .scl_wait_us = 0,
            .flags = 0,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &sensor_config, &sensor_handle));

        xSemaphoreTake(i2c_bus_init_done, portMAX_DELAY);

        ESP_ERROR_CHECK(bme.init(i2c_bus_handle));
        bme.start();

        // preliminary setup
        bme.get_sensor_settings();

        xSemaphoreGive(sensor_init_done);
        vTaskDelete(nullptr);
    }

    void vBlinkTask(void* pvParameters) {
        AppGpioOut led(GPIO_NUM_2);
        ESP_ERROR_CHECK(led.init());
        while (true) {
            led.set(!led.get());
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    };

    void vGetTempTask(void* pvParameters) {
        const char* tag = "vGetTempTask";
        xSemaphoreTake(sensor_init_done, portMAX_DELAY);

        // TODO: This is where we add code to read the sensor values
        bme.settings.filter = BME280_FILTER_COEFF_2;
        bme.settings.osr_h = BME280_OVERSAMPLING_1X;
        bme.settings.osr_p = BME280_OVERSAMPLING_1X;
        bme.settings.osr_t = BME280_OVERSAMPLING_1X;
        bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
        bme.set_sensor_settings(BME280_ALL);
        bme.set_sensor_mode(BME280_POWERMODE_NORMAL);
        uint32_t max_delay_us;
        bme.cal_meas_delay(max_delay_us);
        ESP_LOGD(tag, "max_delay=%lu (us)", max_delay_us);
        uint32_t tick_delay_ms = max_delay_us / 1000;
        vTaskDelay(pdMS_TO_TICKS(tick_delay_ms));
        std::vector<uint8_t> val(1, 0x00);
        bme.read_reg(BME280_REG_STATUS, val);
        uint8_t sensor_comp = {BME280_ALL};
        bme280_data comp_data;
        bme.get_sensor_data(sensor_comp, comp_data);
        ESP_LOGD(tag, "temperature=%f", comp_data.temperature);
        while (true) {
            bme.read_reg(BME280_REG_STATUS, val);
            if (val[0] & BME280_STATUS_MEAS_DONE) {
                vTaskDelay(pdMS_TO_TICKS(tick_delay_ms));
                bme.get_sensor_data(sensor_comp, comp_data);
                ESP_LOGD(tag, "temperature=%f, pressure=%f, humidity=%f", comp_data.temperature, comp_data.pressure, comp_data.humidity);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    };

    void app_main() {
        const char* tag = "app_main";
        printf("ESP-IDF version: %s\n", esp_get_idf_version());

        // Create semaphores
        i2c_bus_init_done = xSemaphoreCreateBinary();
        sensor_init_done = xSemaphoreCreateBinary();

        // insure the semaphore creation was successful
        if (i2c_bus_init_done == nullptr || sensor_init_done == nullptr) {
            ESP_LOGE(tag, "Semaphore creation failed.");
            vTaskSuspend(nullptr);
            return;
        }

        // create a task to blink the LED
        xTaskCreate(vTaskInitI2c, "init_i2c", 4096, nullptr, 4, nullptr);
        xTaskCreate(vTaskInitSensor, "init_sensor", 4096, nullptr, 3, nullptr);
        xTaskCreate(vGetTempTask, "get_temp_task", 4096, nullptr, 2, nullptr);
        xTaskCreate(vBlinkTask, "blink_task", 4096, nullptr, 1, nullptr);

        vTaskSuspend(nullptr);

    };
};
