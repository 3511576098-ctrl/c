#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"   // 你是什么系列就改成 stm32f1xx_hal.h / stm32f4xx_hal.h 等
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ====== I2C address (7-bit) ======
#define MPU6050_I2C_ADDR_7BIT   (0x68u)  // AD0=GND
#define MPU6050_I2C_ADDR_8BIT   (MPU6050_I2C_ADDR_7BIT << 1)

// ====== Registers ======
#define MPU6050_RA_SMPLRT_DIV   0x19
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_INT_ENABLE   0x38
#define MPU6050_RA_INT_PIN_CFG  0x37
#define MPU6050_RA_USER_CTRL    0x6A
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_PWR_MGMT_2   0x6C
#define MPU6050_RA_WHO_AM_I     0x75

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_TEMP_OUT_H   0x41
#define MPU6050_RA_GYRO_XOUT_H  0x43

// ====== Bits ======
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_ACONFIG_AFS_SEL_BIT     4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH  2

// ====== Enums ======
typedef enum {
    MPU6050_CLOCK_INTERNAL = 0,
    MPU6050_CLOCK_PLL_XGYRO = 1,
    MPU6050_CLOCK_PLL_YGYRO = 2,
    MPU6050_CLOCK_PLL_ZGYRO = 3
} mpu6050_clock_t;

typedef enum {
    MPU6050_GYRO_FS_250  = 0,
    MPU6050_GYRO_FS_500  = 1,
    MPU6050_GYRO_FS_1000 = 2,
    MPU6050_GYRO_FS_2000 = 3
} mpu6050_gyro_fs_t;

typedef enum {
    MPU6050_ACCEL_FS_2  = 0,
    MPU6050_ACCEL_FS_4  = 1,
    MPU6050_ACCEL_FS_8  = 2,
    MPU6050_ACCEL_FS_16 = 3
} mpu6050_accel_fs_t;

// ====== Data structs ======
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_vec3_i16_t;

typedef struct {
    mpu6050_vec3_i16_t accel_raw;
    mpu6050_vec3_i16_t gyro_raw;
    int16_t temp_raw;
} mpu6050_raw14_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;          // 7-bit address, normally 0x68
    uint32_t timeout_ms;    // HAL timeout
} mpu6050_t;

// ====== API ======
HAL_StatusTypeDef MPU6050_Init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7, uint32_t timeout_ms);

HAL_StatusTypeDef MPU6050_Reset(mpu6050_t *dev);
HAL_StatusTypeDef MPU6050_SetClockSource(mpu6050_t *dev, mpu6050_clock_t src);
HAL_StatusTypeDef MPU6050_SetSleepEnabled(mpu6050_t *dev, uint8_t enabled);

HAL_StatusTypeDef MPU6050_SetFullScaleGyroRange(mpu6050_t *dev, mpu6050_gyro_fs_t range);
HAL_StatusTypeDef MPU6050_SetFullScaleAccelRange(mpu6050_t *dev, mpu6050_accel_fs_t range);
HAL_StatusTypeDef MPU6050_SetLPF(mpu6050_t *dev, uint16_t lpf_hz);
HAL_StatusTypeDef MPU6050_SetRate(mpu6050_t *dev, uint16_t rate_hz);

HAL_StatusTypeDef MPU6050_ReadWhoAmI(mpu6050_t *dev, uint8_t *whoami);
uint8_t MPU6050_TestConnection(mpu6050_t *dev); // returns 1 ok / 0 fail

HAL_StatusTypeDef MPU6050_ReadAccelRaw(mpu6050_t *dev, mpu6050_vec3_i16_t *acc);
HAL_StatusTypeDef MPU6050_ReadGyroRaw(mpu6050_t *dev, mpu6050_vec3_i16_t *gyro);
HAL_StatusTypeDef MPU6050_ReadTempRaw(mpu6050_t *dev, int16_t *temp_raw);
HAL_StatusTypeDef MPU6050_ReadRaw14(mpu6050_t *dev, mpu6050_raw14_t *out);

#ifdef __cplusplus
}
#endif

#endif
