/* ============================================================
 *  fc_minimal.c —— 四轴飞控最小完整实现
 *
 *  控制结构：
 *      角度外环 (P)  →  角速度内环 (PID)  →  X 型混控  →  PWM
 *
 *  传感器：
 *      MPU6050 (I2C1, 默认地址 0x68)
 *      陀螺量程 ±500 dps，加速度量程 ±2 g
 *
 *  电机映射（与原理图一致）：
 *      M1 = TIM3_CH1 = PA6
 *      M2 = TIM3_CH2 = PA7
 *      M3 = TIM3_CH3 = PB0
 *      M4 = TIM3_CH4 = PB1
 *
 *  约定：所有角度单位弧度，角速度弧度/秒，油门 / 电机输出 0~1。
 * ============================================================ */

#include "fc_minimal.h"
#include "i2c.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------
 *  常量
 * ------------------------------------------------------------ */
#define DEG2RAD              0.017453292519943295f
#define RAD2DEG              57.29577951308232f

/* MPU6050 当前配置下的刻度因子 */
#define GYRO_LSB_PER_DPS     65.5f       /* ±500dps 对应 65.5 LSB/(°/s) */
#define ACCEL_LSB_PER_G      16384.0f    /* ±2g    对应 16384 LSB/g    */

/* 解锁后的怠速油门：低于此值不跑 PID，电机熄火（防止地面误触发） */
#define IDLE_THROTTLE        0.05f

/* IMU 连续读失败计数到达此值时进入失控保护 */
#define IMU_FAIL_LIMIT       20

/* 陀螺校准采样数（每个采样间隔 2ms，共约 1 秒） */
#define GYRO_CAL_SAMPLES     500

/* 期望姿态最大角度限幅：±30° */
#define MAX_TILT_RAD         (30.0f * DEG2RAD)
/* 期望偏航角速度最大限幅：±180°/s */
#define MAX_YAW_RATE_RPS     (180.0f * DEG2RAD)

/* ------------------------------------------------------------
 *  全局状态
 * ------------------------------------------------------------ */
mpu6050_t  g_mpu = {0};
Attitude_t g_att = {0};
Setpoint_t g_sp  = {0};

static FC_Mode_t s_mode = FC_MODE_DISARMED;
static GyroCal_t s_cal  = {0};
static uint8_t   s_imu_fail_cnt = 0;

/* PID 初值：先给一组保守值，实际飞行需自己整定 */
static PID_t pid_roll = {
    .kp = 0.08f, .ki = 0.03f, .kd = 0.002f,
    .i = 0.0f, .i_limit = 0.25f, .out_limit = 0.35f,
    .prev_meas = 0.0f, .d = 0.0f, .d_lpf_alpha = 0.2f
};
static PID_t pid_pitch = {
    .kp = 0.08f, .ki = 0.03f, .kd = 0.002f,
    .i = 0.0f, .i_limit = 0.25f, .out_limit = 0.35f,
    .prev_meas = 0.0f, .d = 0.0f, .d_lpf_alpha = 0.2f
};
static PID_t pid_yaw = {
    .kp = 0.06f, .ki = 0.02f, .kd = 0.000f,
    .i = 0.0f, .i_limit = 0.20f, .out_limit = 0.25f,
    .prev_meas = 0.0f, .d = 0.0f, .d_lpf_alpha = 0.2f
};

/* 角度外环 P 系数：把角度误差(rad) 转换为期望角速度(rad/s) */
static float kp_angle = 4.0f;

/* ------------------------------------------------------------
 *  通用工具
 * ------------------------------------------------------------ */
static inline float clampf(float x, float mn, float mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

static inline float gyro_raw_to_rads(int16_t raw)
{
    return ((float)raw / GYRO_LSB_PER_DPS) * DEG2RAD;
}

static inline float accel_raw_to_g(int16_t raw)
{
    return (float)raw / ACCEL_LSB_PER_G;
}

/* ------------------------------------------------------------
 *  PWM 输出
 * ------------------------------------------------------------ */
static inline uint16_t duty_to_ccr(TIM_HandleTypeDef *htim, float duty01)
{
    duty01 = clampf(duty01, 0.0f, 1.0f);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    return (uint16_t)(duty01 * (float)arr);
}

static void motors_write(float m1, float m2, float m3, float m4)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_to_ccr(&htim3, m1));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_to_ccr(&htim3, m2));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty_to_ccr(&htim3, m3));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty_to_ccr(&htim3, m4));
}

static void motors_off(void)
{
    motors_write(0.0f, 0.0f, 0.0f, 0.0f);
}

/* ------------------------------------------------------------
 *  陀螺零偏校准（开机静止取均值）
 *  返回 HAL_OK 表示成功，HAL_ERROR 表示读 IMU 失败
 * ------------------------------------------------------------ */
static HAL_StatusTypeDef gyro_calibrate(uint16_t samples)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    mpu6050_raw14_t raw;

    for (uint16_t i = 0; i < samples; ++i) {
        if (MPU6050_ReadRaw14(&g_mpu, &raw) != HAL_OK) {
            return HAL_ERROR;
        }
        sx += gyro_raw_to_rads(raw.gyro_raw.x);
        sy += gyro_raw_to_rads(raw.gyro_raw.y);
        sz += gyro_raw_to_rads(raw.gyro_raw.z);
        HAL_Delay(2);
    }
    s_cal.gx_bias    = sx / (float)samples;
    s_cal.gy_bias    = sy / (float)samples;
    s_cal.gz_bias    = sz / (float)samples;
    s_cal.calibrated = 1;
    return HAL_OK;
}

/* ------------------------------------------------------------
 *  互补滤波姿态解算
 *      短期信赖陀螺（响应快），长期靠加速度修正（无漂移）。
 *      yaw 没有磁力计参考，纯陀螺积分，会缓慢漂移，属正常现象。
 * ------------------------------------------------------------ */
static void attitude_update(Attitude_t *att,
                            float ax_g, float ay_g, float az_g,
                            float gx, float gy, float gz,
                            float dt)
{
    /* 角速度直通，给内环 PID 用 */
    att->gyro_roll  = gx;
    att->gyro_pitch = gy;
    att->gyro_yaw   = gz;

    /* 加速度归一化，去掉幅值影响（缓解振动/重力变化的扰动） */
    float norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (norm > 1e-6f) {
        ax_g /= norm;
        ay_g /= norm;
        az_g /= norm;
    }

    /* 由加速度计算 roll / pitch（弧度） */
    float roll_acc  = atan2f(ay_g, az_g);
    float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    /* 互补滤波系数：tau 越大越信任陀螺
       dt = 0.002 时 alpha ≈ 0.996 */
    const float tau   = 0.5f;
    float alpha       = tau / (tau + dt);
    if (alpha > 0.999f) alpha = 0.999f;
    if (alpha < 0.5f)   alpha = 0.5f;

    att->roll  = alpha * (att->roll  + gx * dt) + (1.0f - alpha) * roll_acc;
    att->pitch = alpha * (att->pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
    att->yaw  += gz * dt;
}

/* ------------------------------------------------------------
 *  角速度内环 PID
 *      D 作用于测量值（D-on-measurement），避免 setpoint 跳变
 *      时 D 项产生瞬时冲击。
 * ------------------------------------------------------------ */
static float pid_rate_update(PID_t *pid, float rate_sp, float rate_meas, float dt)
{
    float err = rate_sp - rate_meas;

    /* P */
    float p = pid->kp * err;

    /* I（积分限幅，防止饱和） */
    pid->i += pid->ki * err * dt;
    pid->i  = clampf(pid->i, -pid->i_limit, pid->i_limit);

    /* D on measurement，并经一阶低通滤波 */
    float meas_dot   = (rate_meas - pid->prev_meas) / dt;
    pid->prev_meas   = rate_meas;
    float d_raw      = -pid->kd * meas_dot;
    pid->d          += pid->d_lpf_alpha * (d_raw - pid->d);

    float out = p + pid->i + pid->d;
    return clampf(out, -pid->out_limit, pid->out_limit);
}

static void pid_reset(PID_t *pid)
{
    pid->i         = 0.0f;
    pid->d         = 0.0f;
    pid->prev_meas = 0.0f;
}

/* ------------------------------------------------------------
 *  X 型混控
 *
 *      M1 (前左, CW)        M2 (前右, CCW)
 *               \    /
 *                \  /
 *                 ××
 *                /  \
 *               /    \
 *      M4 (后左, CCW)       M3 (后右, CW)
 *
 *      pitch+ : 抬头 → 后侧加速、前侧减速
 *      roll+  : 右倾 → 左侧加速、右侧减速
 *      yaw+   : 顺时针偏航 → CCW 桨加速、CW 桨减速
 *
 *  注：电机布局/旋向与你的实际机体若不一致，需对调相应符号。
 * ------------------------------------------------------------ */
static void mixer_quad_x(float thr, float u_roll, float u_pitch, float u_yaw,
                         MotorMix_t *out)
{
    out->m1 = thr - u_pitch + u_roll + u_yaw;   /* 前左 CW   */
    out->m2 = thr - u_pitch - u_roll - u_yaw;   /* 前右 CCW  */
    out->m3 = thr + u_pitch - u_roll + u_yaw;   /* 后右 CW   */
    out->m4 = thr + u_pitch + u_roll - u_yaw;   /* 后左 CCW  */

    out->m1 = clampf(out->m1, 0.0f, 1.0f);
    out->m2 = clampf(out->m2, 0.0f, 1.0f);
    out->m3 = clampf(out->m3, 0.0f, 1.0f);
    out->m4 = clampf(out->m4, 0.0f, 1.0f);
}

/* ============================================================
 *  解锁 / 上锁 / 失控保护
 * ============================================================ */
void FC_Arm(void)
{
    if (s_mode != FC_MODE_DISARMED) return;     /* 仅允许从 DISARMED 进入 ARMED */
    if (!s_cal.calibrated)          return;     /* 未完成陀螺校准禁止解锁     */
    if (g_sp.throttle > 0.02f)      return;     /* 油门未归零禁止解锁         */

    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);
    s_mode = FC_MODE_ARMED;
}

void FC_Disarm(void)
{
    s_mode        = FC_MODE_DISARMED;
    g_sp.throttle = 0.0f;
    motors_off();
}

void FC_Failsafe(void)
{
    s_mode        = FC_MODE_FAILSAFE;
    g_sp.throttle = 0.0f;
    motors_off();
}

FC_Mode_t FC_GetMode(void)
{
    return s_mode;
}

/* ============================================================
 *  设定值入口（统一限幅）
 * ============================================================ */
void FC_SetThrottle(float throttle_0_1)
{
    g_sp.throttle = clampf(throttle_0_1, 0.0f, 1.0f);
}

void FC_SetAttitude(float roll_rad, float pitch_rad, float yaw_rate_rps)
{
    g_sp.roll     = clampf(roll_rad,     -MAX_TILT_RAD,     MAX_TILT_RAD);
    g_sp.pitch    = clampf(pitch_rad,    -MAX_TILT_RAD,     MAX_TILT_RAD);
    g_sp.yaw_rate = clampf(yaw_rate_rps, -MAX_YAW_RATE_RPS, MAX_YAW_RATE_RPS);
}

/* ============================================================
 *  上电初始化
 *      1) 启动 4 路 PWM 并清零
 *      2) 初始化 MPU6050 (I2C1)
 *      3) 提高采样率到 500Hz
 *      4) 陀螺零偏校准（机体必须静止）
 *      5) 用首帧加速度初始化姿态角，避免起飞瞬间收敛慢
 * ============================================================ */
void FC_Init(void)
{
    /* 1) PWM 启动 + 输出清零（防止上电瞬间桨转） */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    motors_off();

    /* 2) MPU6050 初始化（如不在 I2C1 上请改为 &hi2c2） */
    if (MPU6050_Init(&g_mpu, &hi2c1, MPU6050_I2C_ADDR_7BIT, 500) != HAL_OK) {
        FC_Failsafe();
        return;
    }

    /* 3) 调整采样率与控制环匹配 */
    MPU6050_SetRate(&g_mpu, 500);

    /* 4) 清空姿态与设定值 */
    memset(&g_att, 0, sizeof(g_att));
    memset(&g_sp,  0, sizeof(g_sp));

    /* 5) 陀螺零偏校准 */
    s_mode = FC_MODE_CALIBRATING;
    if (gyro_calibrate(GYRO_CAL_SAMPLES) != HAL_OK) {
        FC_Failsafe();
        return;
    }

    /* 6) 首帧加速度初始化 roll / pitch */
    mpu6050_raw14_t raw;
    if (MPU6050_ReadRaw14(&g_mpu, &raw) == HAL_OK) {
        float ax = accel_raw_to_g(raw.accel_raw.x);
        float ay = accel_raw_to_g(raw.accel_raw.y);
        float az = accel_raw_to_g(raw.accel_raw.z);
        g_att.roll  = atan2f(ay, az);
        g_att.pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    }

    /* 7) 校准完成 → 等待外部 FC_Arm() 解锁 */
    s_mode         = FC_MODE_DISARMED;
    s_imu_fail_cnt = 0;
}

/* ============================================================
 *  控制循环：每 dt 秒调用一次（建议 500Hz, dt=0.002）
 *
 *  执行流程：
 *      1) 读 MPU6050（带失败计数 → 失控保护）
 *      2) 单位换算 + 减陀螺零偏
 *      3) 互补滤波更新姿态
 *      4) 油门 < 怠速 → PID 复位、电机熄火
 *      5) 角度外环 P → 期望角速度
 *      6) 角速度内环 PID → 三轴控制量
 *      7) 混控 → PWM
 * ============================================================ */
void FC_Step(float dt)
{
    /* 非 ARMED 状态全部熄火 */
    if (s_mode != FC_MODE_ARMED) {
        motors_off();
        return;
    }

    /* 1) 读 IMU */
    mpu6050_raw14_t raw;
    if (MPU6050_ReadRaw14(&g_mpu, &raw) != HAL_OK) {
        if (++s_imu_fail_cnt > IMU_FAIL_LIMIT) {
            FC_Failsafe();
        } else {
            motors_off();
        }
        return;
    }
    s_imu_fail_cnt = 0;

    /* 2) 单位换算 + 减零偏 */
    float ax = accel_raw_to_g(raw.accel_raw.x);
    float ay = accel_raw_to_g(raw.accel_raw.y);
    float az = accel_raw_to_g(raw.accel_raw.z);
    float gx = gyro_raw_to_rads(raw.gyro_raw.x) - s_cal.gx_bias;
    float gy = gyro_raw_to_rads(raw.gyro_raw.y) - s_cal.gy_bias;
    float gz = gyro_raw_to_rads(raw.gyro_raw.z) - s_cal.gz_bias;

    /* 3) 姿态解算 */
    attitude_update(&g_att, ax, ay, az, gx, gy, gz, dt);

    /* 4) 怠速保护：油门很小时不跑 PID，避免地面振动一直累积积分 */
    if (g_sp.throttle < IDLE_THROTTLE) {
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);
        motors_off();
        return;
    }

    /* 5) 外环：角度误差 → 期望角速度 */
    float rate_sp_roll  = kp_angle * (g_sp.roll  - g_att.roll);
    float rate_sp_pitch = kp_angle * (g_sp.pitch - g_att.pitch);
    float rate_sp_yaw   = g_sp.yaw_rate;

    /* 6) 内环：角速度 PID */
    float u_roll  = pid_rate_update(&pid_roll,  rate_sp_roll,  g_att.gyro_roll,  dt);
    float u_pitch = pid_rate_update(&pid_pitch, rate_sp_pitch, g_att.gyro_pitch, dt);
    float u_yaw   = pid_rate_update(&pid_yaw,   rate_sp_yaw,   g_att.gyro_yaw,   dt);

    /* 7) 混控 + PWM */
    MotorMix_t mix;
    mixer_quad_x(g_sp.throttle, u_roll, u_pitch, u_yaw, &mix);
    motors_write(mix.m1, mix.m2, mix.m3, mix.m4);
}
