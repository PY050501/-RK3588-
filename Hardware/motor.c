#include "motor.h"
#include "Delay.h"
#include <stdlib.h>

// 8拍步进序列（28BYJ-48推荐使用8拍模式，扭矩更大）
// 顺序：IN1 IN2 IN3 IN4
static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},  // A
    {1, 1, 0, 0},  // AB
    {0, 1, 0, 0},  // B
    {0, 1, 1, 0},  // BC
    {0, 0, 1, 0},  // C
    {0, 0, 1, 1},  // CD
    {0, 0, 0, 1},  // D
    {1, 0, 0, 1}   // DA
};

// 当前步数位置和速度
static uint8_t current_step = 0;       // 当前步序索引（0-7）
static uint16_t step_delay = 2;        // 步间延时（ms），默认2ms
static uint8_t current_position = 0;   // 当前齿轮位置（0-31）
static uint16_t steps_per_rev = MOTOR_STEPS_PER_REV;  // 一圈步数（可校准）

/**
 * @brief  步进电机初始化
 * @param  无
 * @retval 无
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(MOTOR_GPIO_CLK, ENABLE);
    
    // 配置4个控制引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = MOTOR_IN1_PIN | MOTOR_IN2_PIN | 
                                  MOTOR_IN3_PIN | MOTOR_IN4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStructure);
    
    // 初始化所有引脚为低电平
    Motor_Stop();
    
    current_position = 0;
}

/**
 * @brief  设置步进电机引脚状态
 * @param  step_index: 步序索引（0-7）
 * @retval 无
 */
static void Motor_SetPins(uint8_t step_index)
{
    // IN1
    if(step_sequence[step_index][0])
        GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_IN1_PIN);
    else
        GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_IN1_PIN);
    
    // IN2
    if(step_sequence[step_index][1])
        GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_IN2_PIN);
    else
        GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_IN2_PIN);
    
    // IN3
    if(step_sequence[step_index][2])
        GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_IN3_PIN);
    else
        GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_IN3_PIN);
    
    // IN4
    if(step_sequence[step_index][3])
        GPIO_SetBits(MOTOR_GPIO_PORT, MOTOR_IN4_PIN);
    else
        GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_IN4_PIN);
}

/**
 * @brief  步进电机转动指定步数
 * @param  steps: 步数
 * @param  dir: 方向（MOTOR_DIR_CW顺时针，MOTOR_DIR_CCW逆时针）
 * @retval 无
 */
void Motor_Step(int32_t steps, MotorDirection dir)
{
    int32_t i;
    
    for(i = 0; i < steps; i++)
    {
        if(dir == MOTOR_DIR_CW)
        {
            // 顺时针：步序递增
            current_step++;
            if(current_step >= 8)
                current_step = 0;
        }
        else
        {
            // 逆时针：步序递减
            if(current_step == 0)
                current_step = 7;
            else
                current_step--;
        }
        
        Motor_SetPins(current_step);
        Delay_ms(step_delay);
    }
}

/**
 * @brief  停止电机（所有线圈断电）
 * @param  无
 * @retval 无
 */
void Motor_Stop(void)
{
    GPIO_ResetBits(MOTOR_GPIO_PORT, MOTOR_IN1_PIN | MOTOR_IN2_PIN | 
                                    MOTOR_IN3_PIN | MOTOR_IN4_PIN);
}

/**
 * @brief  设置电机转速
 * @param  delay_ms: 步间延时（ms），越小速度越快，建议1-10ms
 * @retval 无
 */
void Motor_SetSpeed(uint16_t delay_ms)
{
    if(delay_ms < 1)
        delay_ms = 1;  // 最小1ms
    step_delay = delay_ms;
}

/**
 * @brief  设置一圈步数（用于校准）
 * @param  steps: 实际测量的一圈步数
 * @retval 无
 * @note   默认8192步，如果转动不准确可调整，常见值：8192、8076、4096
 */
void Motor_SetStepsPerRev(uint16_t steps)
{
    if(steps > 0)
        steps_per_rev = steps;
}

/**
 * @brief  按角度旋转
 * @param  angle: 旋转角度
 * @param  dir: 方向
 * @retval 无
 */
void Motor_Rotate(float angle, MotorDirection dir)
{
    int32_t steps = (int32_t)(angle * steps_per_rev / 360.0f);
    Motor_Step(steps, dir);
}

/**
 * @brief  顺时针旋转指定圈数
 * @param  rounds: 圈数（支持小数，如 1.5 表示1.5圈）
 * @retval 无
 */
void Motor_RotateCW_Rounds(float rounds)
{
    int32_t steps = (int32_t)(rounds * steps_per_rev);
    Motor_Step(steps, MOTOR_DIR_CW);
}

/**
 * @brief  逆时针旋转指定圈数
 * @param  rounds: 圈数（支持小数，如 0.5 表示半圈）
 * @retval 无
 */
void Motor_RotateCCW_Rounds(float rounds)
{
    int32_t steps = (int32_t)(rounds * steps_per_rev);
    Motor_Step(steps, MOTOR_DIR_CCW);
}

/**
 * @brief  旋转指定圈数（正数顺时针，负数逆时针）
 * @param  rounds: 圈数（正数顺时针，负数逆时针）
 * @retval 无
 */
void Motor_RotateRounds(int16_t rounds)
{
    if(rounds > 0)
    {
        Motor_RotateCW_Rounds((uint16_t)rounds);
    }
    else if(rounds < 0)
    {
        Motor_RotateCCW_Rounds((uint16_t)(-rounds));
    }
    // rounds == 0 时不做任何动作
}

/**
 * @brief  移动齿轮链条指定齿数
 * @param  teeth: 要移动的齿数（正数前进，负数后退）
 * @retval 无
 */
void ChainGear_MoveTeeth(int16_t teeth)
{
    int32_t steps;
    MotorDirection dir;
    int16_t new_position;
    
    // 计算需要的步数（假设步进电机一圈对应TEETH_PER_ROTATION个齿）
    steps = (int32_t)((float)abs(teeth) * steps_per_rev / TEETH_PER_ROTATION);
    
    // 确定方向
    if(teeth >= 0)
        dir = MOTOR_DIR_CW;
    else
        dir = MOTOR_DIR_CCW;
    
    // 执行转动
    Motor_Step(steps, dir);
    
    // 更新位置
    new_position = (int16_t)current_position + teeth;
    while(new_position < 0)
        new_position += TOTAL_TEETH;
    current_position = (uint8_t)(new_position % TOTAL_TEETH);
}

/**
 * @brief  移动齿轮链条到指定位置
 * @param  position: 目标位置（0-31）
 * @retval 无
 */
void ChainGear_MoveToPosition(uint8_t position)
{
    int16_t teeth_to_move;
    
    // 限制范围
    if(position >= TOTAL_TEETH)
        position = TOTAL_TEETH - 1;
    
    // 计算需要移动的齿数
    teeth_to_move = (int16_t)position - (int16_t)current_position;
    
    // 移动
    ChainGear_MoveTeeth(teeth_to_move);
}

/**
 * @brief  获取当前齿轮位置
 * @param  无
 * @retval 当前位置（0-31）
 */
uint8_t ChainGear_GetPosition(void)
{
    return current_position;
}

/**
 * @brief  复位齿轮链条到初始位置
 * @param  无
 * @retval 无
 */
void ChainGear_Reset(void)
{
    if(current_position != 0)
    {
        ChainGear_MoveToPosition(0);
    }
    Motor_Stop();
}
