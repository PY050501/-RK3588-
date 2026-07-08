#ifndef __STEPPERMOTOR_H
#define __STEPPERMOTOR_H

#include <stdint.h>

// 方向枚举：直接使用 shang / xia 作为实参
typedef enum { shang = 0, xia = 1 } motor_dir_t;

// 参考参数（保持与现有设备一致）
#define STEPS_PER_REV    1600    // 步进电机细分步数
#define SCREW_PITCH      5       // 丝杆螺距(mm)
#define MAX_TRAVEL_MM    1000    // 最大行程(mm) - 仅作参考，不做强制限制
#define HOMING_SPEED_US  50      // 归零速度(参考)
#define NORMAL_SPEED_US  80      // 初始基础速度(脉冲高/低各延时us)

// 引脚定义
#define PUL_PIN          GPIO_Pin_8   // PA8 脉冲（最小系统板通常引出）
#define DIR_PIN          GPIO_Pin_0   // PB0 方向
#define ENA_PIN          GPIO_Pin_1   // PB1 使能（低电平使能）
#define BOTTOM_LIMIT_PIN GPIO_Pin_0   // PE0 底端限位（低电平触发，EXTI0）
#define TOP_LIMIT_PIN    GPIO_Pin_1   // PE1 顶端限位（低电平触发，EXTI1）
#define LIMIT_GPIO_PORT  GPIOE        // 限位开关 GPIO 端口

// 方向电平定义：当DIR为此电平时代表"上(shang)"
// 如果实际运动方向反了，把这个宏从0改为1即可。
#define DIR_SHANG_LEVEL  0  // 1: 高电平为上行，0: 低电平为上行

// 每毫米步数（整数运算，1600/5=320）
#define STEPS_PER_MM     (STEPS_PER_REV / SCREW_PITCH)

// 限位触发后反向逃逸的距离（毫米）
#define LIMIT_ESCAPE_MM  30U

// 函数声明
void StepperMotor_Init(void);
void Motor_Move(motor_dir_t direction, uint32_t distance_mm);
void Motor_SetSpeedUs(uint16_t us);

// 获取运行状态
uint8_t Motor_IsRunning(void);

// 获取限位触发状态并清除标志（用于测试）
uint8_t Motor_GetLimitTriggered(void);

#endif
