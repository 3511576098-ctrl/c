#ifndef __FC_MINIMAL_H__
#define __FC_MINIMAL_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  飞行模式
 *    DISARMED    : 上锁，电机全部熄火
 *    CALIBRATING : 上电后陀螺零偏校准中（必须保持静止）
 *    ARMED       : 解锁，正常飞行
 *    FAILSAFE    : 失控保护（IMU 失效等），电机停转，需重启复位
 * ============================================================ */
typedef enum {
    FC_MODE_DISARMED = 0,
    FC_MODE_CALIBRATING,
    FC_MODE_ARMED,
    FC_MODE_FAILSAFE
} FC_Mode_t;

/* ============================================================
 *  姿态：roll/pitch/yaw 单位为弧度，gyro_* 单位为弧度/秒
 * ============================================================ */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
} Attitude_t;

/* ============================================================
 *  设定值：上层（遥控/串口/上位机）写入的控制目标
 *    roll/pitch     : 期望姿态角，单位弧度
 *    yaw_rate       : 期望偏航角速度（无磁力计，只能控速率），弧度/秒
 *    throttle       : 油门 0~1
 * ============================================================ */
typedef struct {
    float roll;
    float pitch;
    float yaw_rate;
    float throttle;
} Setpoint_t;

/* ============================================================
 *  PID 控制器（用于角速度内环）
 *    i_limit      : 积分限幅，防积分饱和
 *    out_limit    : 输出限幅
 *    d_lpf_alpha  : D 项一阶低通系数 0~1，越小滤波越强
 * ============================================================ */
typedef struct {
    float kp, ki, kd;
    float i;
    float i_limit;
    float out_limit;
    float prev_meas;
    float d;
    float d_lpf_alpha;
} PID_t;

/* 4 路电机输出（0~1） */
typedef struct {
    float m1, m2, m3, m4;
} MotorMix_t;

/* 陀螺零偏校准结果（弧度/秒） */
typedef struct {
    float gx_bias;
    float gy_bias;
    float gz_bias;
    uint8_t calibrated;
} GyroCal_t;

/* ============================================================
 *  对外接口
 * ============================================================ */

/* 上电初始化：启动 PWM、初始化 MPU6050、做陀螺零偏校准。
   阻塞约 1~2 秒，期间务必保持机体静止。 */
void FC_Init(void);

/* 控制循环一拍。dt 为距离上次调用的秒数，建议 500Hz（dt=0.002）。 */
void FC_Step(float dt);

/* 解锁 / 上锁 / 强制失控保护 */
void       FC_Arm(void);
void       FC_Disarm(void);
void       FC_Failsafe(void);
FC_Mode_t  FC_GetMode(void);

/* 上层调用入口：设定油门和姿态目标（带限幅，安全） */
void FC_SetThrottle(float throttle_0_1);
void FC_SetAttitude(float roll_rad, float pitch_rad, float yaw_rate_rps);

/* 全局只读状态：调试 / 上位机回传用 */
extern Setpoint_t g_sp;
extern Attitude_t g_att;
extern mpu6050_t  g_mpu;
extern TIM_HandleTypeDef htim3;   /* 由 tim.c 提供，PWM 输出 */

#ifdef __cplusplus
}
#endif

#endif /* __FC_MINIMAL_H__ */
