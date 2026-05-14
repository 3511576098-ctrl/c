#ifndef __MOTOR_H__
#define __MOTOR_H__



#include "stm32f4xx_hal.h"   

//定义结构体
typedef struct
{
    int32_t count;
    int32_t speed;
} Encoder_t;
typedef struct
{
    int32_t count;
    int32_t speed;
} Output_t;
typedef struct
{
    int32_t speed;
} Target_t;
typedef struct   
{
    Output_t Output;
    Encoder_t Encoder;
	  Target_t target;
} Motor_t;
typedef struct//总结构体
{
    Motor_t MOTOR_A;
    Motor_t MOTOR_B;
	  Motor_t MOTOR_C;
    Motor_t MOTOR_D;
} Robot_t;
typedef struct
{
    float kp;
    float ki;
    float integral;
    float out;
} PI_CONTROLLER;

#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__



/* 你用哪个定时器就改这里 */
#define PWM_TIM   TIM3

/* 四路PWM映射 */
#define PWMA      (PWM_TIM->CCR1)   // TIM3_CH1
#define PWMB      (PWM_TIM->CCR2)   // TIM3_CH2
#define PWMC      (PWM_TIM->CCR3)   // TIM3_CH3
#define PWMD      (PWM_TIM->CCR4)   // TIM3_CH4

/* 读ARR（周期） */
#define PWM_ARR   (PWM_TIM->ARR)

static inline uint16_t PWM_Limit(uint16_t duty)
{
    uint16_t arr = PWM_ARR;
    return (duty > arr) ? arr : duty;
}

#endif



void Motor_Set_pwm();

#endif /* __MOTOR_H__ */
