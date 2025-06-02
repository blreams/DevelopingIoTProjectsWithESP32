#include "AppButton.hpp"
#include "AppLed.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_GPIO GPIO_NUM_33
#define LED_GPIO GPIO_NUM_27

extern "C" void app_main() {
    AppLed led(LED_GPIO);
    auto fn = [&led](bool state) { led.set(state); };
    AppButton button(fn);
    led.init();
    button.init();
    ESP_LOGI("main", "initialized");
    bool ledState = led.get();
    bool curr_state = false;
    while (true) {
        curr_state = led.get();
        if (curr_state != ledState) {
            ledState = curr_state;
            ESP_LOGI("main", "LED state: %s", ledState ? "ON" : "OFF");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskSuspend(nullptr);
}