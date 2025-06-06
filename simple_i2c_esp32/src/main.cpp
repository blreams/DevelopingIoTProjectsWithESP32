#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "AppLed.hpp"
#include "AppI2c.hpp"
#include "AppBme280.hpp"

#define BME280_ADDRESS 0x77

AppLed led(GPIO_NUM_2);
AppI2c i2c(GPIO_NUM_22, GPIO_NUM_21);
AppBme280 bme280(BME280_ADDRESS, &i2c);

extern "C"
{
    void vBlinkTask(void* pvParameters) {
        while (true) {
            led.set(!led.get());
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    };

    void vGetTempTask(void* pvParameters) {
        static const char* TAG = "TTASK";
        SensorReading reading;
        while (true) {
            reading = bme280.read();
            if (reading.success) {
                ESP_LOGI(TAG, "Temperature: %.2f F, Pressure: %.2f inHg, Humidity: %.2f %%",
                    reading.temperature,
                    reading.pressure,
                    reading.humidity
                );
            } else {
                ESP_LOGW(TAG, "Unable to read temp.");
            }
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    };

    void app_main() {
        static const char* TAG = "MAIN";
        ESP_LOGD(TAG, "Hello World!");

        ESP_ERROR_CHECK(led.init());
        ESP_ERROR_CHECK(i2c.init());
        ESP_LOGD(TAG, "I2C initialized.");
        i2c.scan_i2c_bus();
        ESP_ERROR_CHECK(bme280.init());

        // Create a task to blink the LED
        xTaskCreate(vBlinkTask, "blink_task", 4096, nullptr, 5, nullptr);

        // Create a task to read BME280 temperature and log it
        xTaskCreate(vGetTempTask, "bme280_task", 4096, nullptr, 5, nullptr);

        // Suspend the main task now that other tasks are created.
        vTaskSuspend(nullptr);
    }
}