#pragma once


// Definitions for success/error/warning constants
#define BME280_OK                              0
#define BME280_ERR_NULL_PTR                    -1
#define BME280_ERR_COMM_FAIL                   -2
#define BME280_ERR_INVALID_LEN                 -3
#define BME280_ERR_DEV_NOT_FOUND               -4
#define BME280_ERR_SLEEP_MODE_FAIL             -5
#define BME280_ERR_NVM_COPY_FAILED             -6
#define BME280_WARN_INVALID_OSR_MACRO           1

typedef int bme_err_t;

// data constants
#define BME280_CHIP_ID                         0x60

// i2c addresses
#define BME280_I2C_ADDRESS_PRIM                UINT8_C(0x76)
#define BME280_I2C_ADDRESS_SEC                 UINT8_C(0x77)

// named registers
#define BME280_REG_CHIP_ID                     UINT8_C(0xd0)
#define BME280_REG_RESET                       UINT8_C(0xe0)
#define BME280_REG_TEMP_PRESS_CALIB_DATA       UINT8_C(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA         UINT8_C(0xe1)
#define BME280_REG_CTRL_HUM                    UINT8_C(0xf2)
#define BME280_REG_STATUS                      UINT8_C(0xf3)
#define BME280_REG_PWR_CTRL                    UINT8_C(0xf4)
#define BME280_REG_CTRL_MEAS                   UINT8_C(0xf4)
#define BME280_REG_CONFIG                      UINT8_C(0xf5)
#define BME280_REG_DATA                        UINT8_C(0xf7)

// size constants
#define BME280_LEN_TEMP_PRESS_CALIB_DATA       UINT8_C(26)
#define BME280_LEN_HUMIDITY_CALIB_DATA         UINT8_C(7)
#define BME280_LEN_P_T_H_DATA                  UINT8_C(8)

// sensor power modes
#define BME280_POWERMODE_SLEEP                 UINT8_C(0x00)
#define BME280_POWERMODE_FORCED                UINT8_C(0x01)
#define BME280_POWERMODE_NORMAL                UINT8_C(0x03)

#define BME280_SENSOR_MODE_MSK                 UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS                 UINT8_C(0x00)

// soft reset command
#define BME280_SOFT_RESET_COMMAND              UINT8_C(0xb6)

#define BME280_STATUS_IM_UPDATE                UINT8_C(0x01)
#define BME280_STATUS_MEAS_DONE                UINT8_C(0x08)

// sensor component selection constants
#define BME280_PRESS                           UINT8_C(0x01)
#define BME280_TEMP                            UINT8_C(0x02)
#define BME280_HUM                             UINT8_C(0x04)
#define BME280_ALL                             UINT8_C(0x07)

// settings selection constants
#define BME280_SEL_OSR_PRESS                   UINT8_C(0x01)
#define BME280_SEL_OSR_TEMP                    UINT8_C(0x02)
#define BME280_SEL_OSR_HUM                     UINT8_C(0x04)
#define BME280_SEL_FILTER                      UINT8_C(0x08)
#define BME280_SEL_STANDBY                     UINT8_C(0x10)
#define BME280_SEL_ALL_SETTINGS                UINT8_C(0x1F)

#define BME280_OVERSAMPLING_SETTINGS           UINT8_C(0x07)
#define BME280_FILTER_STANDBY_SETTINGS         UINT8_C(0x18)

// oversampling constants
#define BME280_NO_OVERSAMPLING                 UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X                 UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X                 UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X                 UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X                 UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X                UINT8_C(0x05)
#define BME280_OVERSAMPLING_MAX                UINT8_C(16)

#define BME280_CTRL_HUM_MSK                    UINT8_C(0x07)
#define BME280_CTRL_HUM_POS                    UINT8_C(0x00)
#define BME280_CTRL_PRESS_MSK                  UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS                  UINT8_C(0x02)
#define BME280_CTRL_TEMP_MSK                   UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS                   UINT8_C(0x05)

// measurement delay constants
#define BME280_MEAS_OFFSET                     UINT16_C(1250)
#define BME280_MEAS_DUR                        UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET            UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR             UINT16_C(1000)
#define BME280_STARTUP_DELAY                   UINT16_C(2000)

// length constant
#define BME280_MAX_LEN                         UINT8_C(10)

// standby duration selection constants
#define BME280_STANDBY_TIME_0_5_MS             (0x00)
#define BME280_STANDBY_TIME_62_5_MS            (0x01)
#define BME280_STANDBY_TIME_125_MS             (0x02)
#define BME280_STANDBY_TIME_250_MS             (0x03)
#define BME280_STANDBY_TIME_500_MS             (0x04)
#define BME280_STANDBY_TIME_1000_MS            (0x05)
#define BME280_STANDBY_TIME_10_MS              (0x06)
#define BME280_STANDBY_TIME_20_MS              (0x07)

#define BME280_STANDBY_MSK                     UINT8_C(0xE0)
#define BME280_STANDBY_POS                     UINT8_C(0x05)

// bit shift constants
#define BME280_12_BIT_SHIFT                    UINT8_C(12)
#define BME280_8_BIT_SHIFT                     UINT8_C(8)
#define BME280_4_BIT_SHIFT                     UINT8_C(4)

// filter coefficient selection constants
#define BME280_FILTER_COEFF_OFF                (0x00)
#define BME280_FILTER_COEFF_2                  (0x01)
#define BME280_FILTER_COEFF_4                  (0x02)
#define BME280_FILTER_COEFF_8                  (0x03)
#define BME280_FILTER_COEFF_16                 (0x04)

#define BME280_FILTER_MSK                      UINT8_C(0x1C)
#define BME280_FILTER_POS                      UINT8_C(0x02)

/*! @name Macro to SET and GET BITS of a register */
#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))


struct bme280_calibration_data_t {
    uint16_t dig_t1 = 0;
    int16_t dig_t2 = 0;
    int16_t dig_t3 = 0;
    uint16_t dig_p1 = 0;
    int16_t dig_p2 = 0;
    int16_t dig_p3 = 0;
    int16_t dig_p4 = 0;
    int16_t dig_p5 = 0;
    int16_t dig_p6 = 0;
    int16_t dig_p7 = 0;
    int16_t dig_p8 = 0;
    int16_t dig_p9 = 0;
    uint8_t dig_h1 = 0;
    int16_t dig_h2 = 0;
    uint8_t dig_h3 = 0;
    int16_t dig_h4 = 0;
    int16_t dig_h5 = 0;
    int8_t dig_h6 = 0;
    int32_t t_fine = 0;
};

struct bme280_settings_t {
    uint8_t osr_p = 0;
    uint8_t osr_t = 0;
    uint8_t osr_h = 0;
    uint8_t filter = 0;
    uint8_t standby_time = 0;
};

struct bme280_data {
    double pressure;
    double temperature;
    double humidity;
};

struct bme280_uncomp_data {
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
};

struct bme280_dev {
    uint8_t chip_id;
    int8_t intf_rslt = 0;
    struct bme280_calibration_data_t calib_data;
};