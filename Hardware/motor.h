#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

// 28BYJ-48步进电机参数
// 注意：8拍模式下一圈需要8192步，4拍模式下一圈需要4096步
// 当前使用8拍模式：64步×2(半步)×64(减速比) = 8192步
// 实际减速比可能是63.68395:1，所以实际步数约为8192或稍少，可根据实际测量调整
#define MOTOR_STEPS_PER_REV      8192    // 一圈的步数（8拍模式）
#define MOTOR_STEP_ANGLE         0.043945312f  // 每步角度（360/8192）

// ULN2003驱动板引脚定义（使用GPIOB）
#define MOTOR_GPIO_PORT          GPIOB
#define MOTOR_GPIO_CLK           RCC_APB2Periph_GPIOB
#define MOTOR_IN1_PIN            GPIO_Pin_12
#define MOTOR_IN2_PIN            GPIO_Pin_13
#define MOTOR_IN3_PIN            GPIO_Pin_14
#define MOTOR_IN4_PIN            GPIO_Pin_15

// 齿轮链条参数
#define TOTAL_TEETH              32       // 齿轮链条总齿数
#define TEETH_PER_ROTATION       32       // 步进电机转一圈对应的齿数（可调整）

// 转动方向
typedef enum {
    MOTOR_DIR_CW  = 0,  // 顺时针
    MOTOR_DIR_CCW = 1   // 逆时针
} MotorDirection;

// 函数声明
void Motor_Init(void);
void Motor_Step(int32_t steps, MotorDirection dir);
void Motor_Stop(void);
void Motor_SetSpeed(uint16_t delay_ms);
void Motor_SetStepsPerRev(uint16_t steps);      // 设置一圈步数（校准用）
void Motor_Rotate(float angle, MotorDirection dir);
void Motor_RotateCW_Rounds(float rounds);       // 顺时针转指定圈数（支持小数）
void Motor_RotateCCW_Rounds(float rounds);      // 逆时针转指定圈数（支持小数）
void Motor_RotateRounds(int16_t rounds);        // 正数顺时针，负数逆时针
void ChainGear_MoveTeeth(int16_t teeth);
void ChainGear_MoveToPosition(uint8_t position);
uint8_t ChainGear_GetPosition(void);
void ChainGear_Reset(void);

#endif
