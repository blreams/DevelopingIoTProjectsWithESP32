#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "AppMultisensor.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

app::AppMultisensor multisensor;

extern "C"
{
    void vBlinkTask(void* pvParameters) {
        gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
        while (true) {
            gpio_set_level(GPIO_NUM_27, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(GPIO_NUM_27, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    };

    void vGetTempTask(void* pvParameters) {
        static const char* TAG = "BME280";
        // Initialize I2C and BME280 here (pseudo-code)
        app::SensorReading reading;
        while (true) {
            reading = multisensor.read();
            if (reading.success) {
                ESP_LOGI(TAG, "Temperature: %.2f C", reading.temperature);
            } else {
                ESP_LOGW(TAG, "Unable to read temp.");
            }
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    };

    void app_main() {
        static const char* TAG = "MAIN";
        ESP_LOGD(TAG, "Hello World!");

        if (multisensor.init()) {
            ESP_LOGD(TAG, "Multisensor initialized.");
        } else {
            ESP_LOGE(TAG, "Multisensor NOT initialized.");
        }

        // Create a task to blink the LED
        xTaskCreate(vBlinkTask, "blink_task", 4096, nullptr, 5, nullptr);

        // Create a task to read BME280 temperature and log it
        xTaskCreate(vGetTempTask, "bme280_task", 4096, nullptr, 5, nullptr);
    }
}