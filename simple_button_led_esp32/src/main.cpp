#include "AppButton.hpp"
#include "AppLed.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_GPIO GPIO_NUM_33
#define LED_GPIO GPIO_NUM_27

extern "C" void app_main() {
    AppButton button(BUTTON_GPIO);
    button.init();
    AppLed led(LED_GPIO);
    led.init();
    ESP_LOGI("main", "initialized");

    bool btn_state = button.getState();
    while (true) {
        if (btn_state != button.getState()) {
            btn_state = button.getState();
            ESP_LOGI("main", "Changing state");
            led.set(btn_state);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}