#pragma once


// Definitions for error constants
#define BME280_OK                              0
#define BME280_ERR_NULL_PTR                    -1
#define BME280_ERR_COMM_FAIL                   -2
#define BME280_ERR_INVALID_LEN                 -3
#define BME280_ERR_DEV_NOT_FOUND               -4
#define BME280_ERR_SLEEP_MODE_FAIL             -5
#define BME280_ERR_NVM_COPY_FAILED             -6

typedef int bme_err_t;

// data constants
#define BME280_CHIP_ID                         0x60
#define BME280_SOFT_RESET_COMMAND              0xb6
#define BME280_STATUS_IM_UPDATE                UINT8_C(0x01)

// measurement delay constants
#define BME280_MEAS_OFFSET                     UINT16_C(1250)
#define BME280_MEAS_DUR                        UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET            UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR             UINT16_C(1000)
#define BME280_STARTUP_DELAY                   UINT16_C(2000)

// named registers
#define BME280_REG_CHIP_ID                     0xd0
#define BME280_REG_RESET                       0xe0
#define BME280_REG_TEMP_PRESS_CALIB_DATA       0x88
#define BME280_REG_HUMIDITY_CALIB_DATA         0xe1
#define BME280_REG_CTRL_HUM                    0xf2
#define BME280_REG_STATUS                      0xf3
#define BME280_REG_PWR_CTRL                    0xf4
#define BME280_REG_CTRL_MEAS                   0xf4
#define BME280_REG_CONFIG                      0xf5
#define BME280_REG_DATA                        0xf7