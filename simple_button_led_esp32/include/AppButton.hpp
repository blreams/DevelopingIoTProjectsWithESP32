#pragma once
#include <functional>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class AppButton {
private:
    volatile bool state;
    std::function<void(bool)> pressedHandler;

public:
    AppButton(std::function<void(bool)> h);
    void init();
    void toggle();
};
