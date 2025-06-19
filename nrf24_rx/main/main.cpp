#include <iostream>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mirf.h"

#include "sdkconfig.h"

#include "AppGpioOut.hpp"

struct __attribute__((packed)) PacketData {
    double distance;
    double average_distance;
    bool in_range;
    int id;
};

extern "C" {

void blink(void* pvParameters) {
    AppGpioOut led(GPIO_NUM_2);
    ESP_ERROR_CHECK(led.init());
    while (true) {
        led.set(!led.get());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
};

void receiver(void* pvParameters) {
    const char* tag {pcTaskGetName(nullptr)};

    ESP_LOGI(tag, "Starting NRF24L01 Receiver\n");

    // Set SPI pins and initialize NRF24L01
    NRF24_t dev;
    dev.cePin = 26;
    dev.csnPin = 5;
    Nrf24_init(&dev);
    uint8_t payload = 32;
    uint8_t channel = 78;
    Nrf24_config(&dev, channel, payload);

    // Set RX address to match transmitter
    esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"TTTTT");
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "nrf24l01 not installed");
        while (true) { vTaskDelay(1); }
    }
    Nrf24_SetSpeedDataRates(&dev, 1);

    Nrf24_printDetails(&dev);
    ESP_LOGI(tag, "NRF24L01 Receiver ready\n");

    // clear RX fifo
    uint8_t buf[32];
    while (true) {
        if (Nrf24_dataReady(&dev) == false) { break; }
        Nrf24_getData(&dev, buf);
    }

    // get received data as it becomes available
    std::string s;
    struct PacketData packet;
    while (true) {
        // wait for received data
        if (Nrf24_dataReady(&dev)) {
            Nrf24_getData(&dev, buf);
            memcpy(&packet, buf, sizeof(packet));
            ESP_LOGI(tag, "id: %d, distance: %f, average_distance: %f, in_range: %s",
                packet.id, packet.distance, packet.average_distance, packet.in_range ? "true" : "false"
            );
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

void app_main(void) {
    xTaskCreate(blink, "blink", 4096, nullptr, 4, nullptr);
    xTaskCreate(receiver, "receiver", 4096, nullptr, 4, nullptr);

    vTaskSuspend(nullptr);
};

}