#include "mpu6050.h"

// ---------- internal helpers ----------
static HAL_StatusTypeDef mpu_read(mpu6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c,
                            (uint16_t)(dev->addr7 << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf, len,
                            dev->timeout_ms);
}

static HAL_StatusTypeDef mpu_write(mpu6050_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(dev->hi2c,
                             (uint16_t)(dev->addr7 << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t*)buf, len,
                             dev->timeout_ms);
}

static HAL_StatusTypeDef mpu_read_u8(mpu6050_t *dev, uint8_t reg, uint8_t *val)
{
    return mpu_read(dev, reg, val, 1);
}

static HAL_StatusTypeDef mpu_write_u8(mpu6050_t *dev, uint8_t reg, uint8_t val)
{
    return mpu_write(dev, reg, &val, 1);
}

static HAL_StatusTypeDef mpu_write_bits(mpu6050_t *dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    // read-modify-write
    uint8_t b = 0;
    HAL_StatusTypeDef st = mpu_read_u8(dev, reg, &b);
    if (st != HAL_OK) return st;

    uint8_t mask = (uint8_t)(((1u << length) - 1u) << (bitStart - length + 1u));
    data <<= (bitStart - length + 1u);
    data &= mask;
    b &= (uint8_t)(~mask);
    b |= data;

    return mpu_write_u8(dev, reg, b);
}

static HAL_StatusTypeDef mpu_write_bit(mpu6050_t *dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b = 0;
    HAL_StatusTypeDef st = mpu_read_u8(dev, reg, &b);
    if (st != HAL_OK) return st;

    if (data) b |=  (uint8_t)(1u << bitNum);
    else      b &= (uint8_t)~(1u << bitNum);

    return mpu_write_u8(dev, reg, b);
}

// ---------- public API ----------
HAL_StatusTypeDef MPU6050_Init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7, uint32_t timeout_ms)
{
    if (!dev || !hi2c) return HAL_ERROR;

    dev->hi2c = hi2c;
    dev->addr7 = addr7;
    dev->timeout_ms = timeout_ms;

    // Basic bring-up sequence similar to your SPL code
    HAL_StatusTypeDef st;

    st = MPU6050_Reset(dev);
    if (st != HAL_OK) return st;
    HAL_Delay(200);

    // wake up
    st = mpu_write_u8(dev, MPU6050_RA_PWR_MGMT_1, 0x00);
    if (st != HAL_OK) return st;

    // Check ID
    uint8_t id = 0;
    st = MPU6050_ReadWhoAmI(dev, &id);
    if (st != HAL_OK) return st;
    if (id != 0x68) return HAL_ERROR;

    // Clock source: PLL X gyro (common)
    st = MPU6050_SetClockSource(dev, MPU6050_CLOCK_PLL_XGYRO);
    if (st != HAL_OK) return st;

    // Typical config: gyro 500dps, accel 2g
    st = MPU6050_SetFullScaleGyroRange(dev, MPU6050_GYRO_FS_500);
    if (st != HAL_OK) return st;

    st = MPU6050_SetFullScaleAccelRange(dev, MPU6050_ACCEL_FS_2);
    if (st != HAL_OK) return st;

    // Disable interrupts/FIFO/master like your code
    st = mpu_write_u8(dev, MPU6050_RA_INT_ENABLE, 0x00);
    if (st != HAL_OK) return st;

    st = mpu_write_u8(dev, MPU6050_RA_USER_CTRL, 0x00);
    if (st != HAL_OK) return st;

    st = mpu_write_u8(dev, MPU6050_RA_INT_PIN_CFG, 0x80); // bypass enabled
    if (st != HAL_OK) return st;

    // Set rate (example 50Hz, you can change later)
    st = MPU6050_SetRate(dev, 50);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Reset(mpu6050_t *dev)
{
    // reset device
    return mpu_write_u8(dev, MPU6050_RA_PWR_MGMT_1, 0x80);
}

HAL_StatusTypeDef MPU6050_SetClockSource(mpu6050_t *dev, mpu6050_clock_t src)
{
    return mpu_write_bits(dev,
                          MPU6050_RA_PWR_MGMT_1,
                          MPU6050_PWR1_CLKSEL_BIT,
                          MPU6050_PWR1_CLKSEL_LENGTH,
                          (uint8_t)src);
}

HAL_StatusTypeDef MPU6050_SetSleepEnabled(mpu6050_t *dev, uint8_t enabled)
{
    return mpu_write_bit(dev, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled ? 1u : 0u);
}

HAL_StatusTypeDef MPU6050_SetFullScaleGyroRange(mpu6050_t *dev, mpu6050_gyro_fs_t range)
{
    return mpu_write_bits(dev,
                          MPU6050_RA_GYRO_CONFIG,
                          MPU6050_GCONFIG_FS_SEL_BIT,
                          MPU6050_GCONFIG_FS_SEL_LENGTH,
                          (uint8_t)range);
}

HAL_StatusTypeDef MPU6050_SetFullScaleAccelRange(mpu6050_t *dev, mpu6050_accel_fs_t range)
{
    return mpu_write_bits(dev,
                          MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_AFS_SEL_BIT,
                          MPU6050_ACONFIG_AFS_SEL_LENGTH,
                          (uint8_t)range);
}

// Map LPF Hz to DLPF_CFG (same mapping as many drivers)
// 188->1, 98->2, 42->3, 20->4, 10->5, else->6
HAL_StatusTypeDef MPU6050_SetLPF(mpu6050_t *dev, uint16_t lpf_hz)
{
    uint8_t cfg = 0;
    if (lpf_hz >= 188) cfg = 1;
    else if (lpf_hz >= 98) cfg = 2;
    else if (lpf_hz >= 42) cfg = 3;
    else if (lpf_hz >= 20) cfg = 4;
    else if (lpf_hz >= 10) cfg = 5;
    else cfg = 6;

    return mpu_write_u8(dev, MPU6050_RA_CONFIG, cfg);
}

// Fs = 1kHz when DLPF enabled; sample rate = 1kHz/(1+SMPLRT_DIV)
HAL_StatusTypeDef MPU6050_SetRate(mpu6050_t *dev, uint16_t rate_hz)
{
    if (rate_hz > 1000) rate_hz = 1000;
    if (rate_hz < 4) rate_hz = 4;

    uint8_t div = (uint8_t)(1000u / rate_hz - 1u);
    HAL_StatusTypeDef st = mpu_write_u8(dev, MPU6050_RA_SMPLRT_DIV, div);
    if (st != HAL_OK) return st;

    // Common trick: LPF = rate/2
    return MPU6050_SetLPF(dev, (uint16_t)(rate_hz / 2u));
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(mpu6050_t *dev, uint8_t *whoami)
{
    if (!whoami) return HAL_ERROR;
    return mpu_read_u8(dev, MPU6050_RA_WHO_AM_I, whoami);
}

uint8_t MPU6050_TestConnection(mpu6050_t *dev)
{
    uint8_t id = 0;
    if (MPU6050_ReadWhoAmI(dev, &id) != HAL_OK) return 0;
    return (id == 0x68) ? 1u : 0u;
}

HAL_StatusTypeDef MPU6050_ReadAccelRaw(mpu6050_t *dev, mpu6050_vec3_i16_t *acc)
{
    if (!acc) return HAL_ERROR;
    uint8_t buf[6];
    HAL_StatusTypeDef st = mpu_read(dev, MPU6050_RA_ACCEL_XOUT_H, buf, 6);
    if (st != HAL_OK) return st;

    acc->x = (int16_t)((buf[0] << 8) | buf[1]);
    acc->y = (int16_t)((buf[2] << 8) | buf[3]);
    acc->z = (int16_t)((buf[4] << 8) | buf[5]);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadGyroRaw(mpu6050_t *dev, mpu6050_vec3_i16_t *gyro)
{
    if (!gyro) return HAL_ERROR;
    uint8_t buf[6];
    HAL_StatusTypeDef st = mpu_read(dev, MPU6050_RA_GYRO_XOUT_H, buf, 6);
    if (st != HAL_OK) return st;

    gyro->x = (int16_t)((buf[0] << 8) | buf[1]);
    gyro->y = (int16_t)((buf[2] << 8) | buf[3]);
    gyro->z = (int16_t)((buf[4] << 8) | buf[5]);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadTempRaw(mpu6050_t *dev, int16_t *temp_raw)
{
    if (!temp_raw) return HAL_ERROR;
    uint8_t buf[2];
    HAL_StatusTypeDef st = mpu_read(dev, MPU6050_RA_TEMP_OUT_H, buf, 2);
    if (st != HAL_OK) return st;

    *temp_raw = (int16_t)((buf[0] << 8) | buf[1]);
    return HAL_OK;
}

// Burst read: accel(6) + temp(2) + gyro(6) = 14 bytes
HAL_StatusTypeDef MPU6050_ReadRaw14(mpu6050_t *dev, mpu6050_raw14_t *out)
{
    if (!out) return HAL_ERROR;

    uint8_t buf[14];
    HAL_StatusTypeDef st = mpu_read(dev, MPU6050_RA_ACCEL_XOUT_H, buf, 14);
    if (st != HAL_OK) return st;

    out->accel_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_raw.z = (int16_t)((buf[4] << 8) | buf[5]);

    out->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);

    out->gyro_raw.x = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_raw.y = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_raw.z = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}
