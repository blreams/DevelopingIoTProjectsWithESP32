#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "BME280Wrapper.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

BME280Wrapper bme;

void i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

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
        float temp, press, hum;
        while (true) {
            if (bme.readSensorData(temp, press, hum)) {
                ESP_LOGI(TAG, "Temperature: %.2f C", temp);
            } else {
                ESP_LOGW(TAG, "Unable to read temp.");
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    };

    void app_main() {
        static const char* TAG = "MAIN";
        ESP_LOGD(TAG, "Hello World!");

        i2c_master_init();
        if (bme.begin()) {
            ESP_LOGD(TAG, "BME280 initialized.");
        } else {
            ESP_LOGW(TAG, "BME280 NOT initialized.");
        }

        ESP_LOGD(TAG, "Scanning i2c bus...");
        for (uint8_t addr = 1; addr < 127; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                ESP_LOGD("I2C", "Found device at 0x%02X", addr);
            }
        }
        ESP_LOGD(TAG, "Scan complete.");


        // Create a task to blink the LED
        xTaskCreate(vBlinkTask, "blink_task", 4096, nullptr, 5, nullptr);

        // Create a task to read BME280 temperature and log it
        xTaskCreate(vGetTempTask, "bme280_task", 4096, nullptr, 5, nullptr);
    }
}