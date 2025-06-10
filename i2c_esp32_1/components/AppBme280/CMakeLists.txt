idf_component_register(
    SRCS "AppBme280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES AppI2cMaster AppGpioOut
)