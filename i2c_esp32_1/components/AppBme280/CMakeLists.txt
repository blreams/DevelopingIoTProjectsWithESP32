idf_component_register(
    SRCS "AppBme280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c AppGpioOut
)