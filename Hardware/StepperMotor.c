#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "Delay.h"
#include "StepperMotor.h"

// 前置声明
static void Motor_GPIO_Init(void);
static void Motor_EXTI_Init(void);
static void Motor_Enable(uint8_t enable);
static void Motor_SetDirection_Shang(void);
static void Motor_SetDirection_Xia(void);
static void Motor_PulseSteps(uint32_t steps, uint32_t delay_us);
// 同步速度脉冲（自动使用当前全局速度并做简单加速）
static void Motor_PulseSteps_Sync(uint32_t steps);

// 运行状态标志（供中断与主循环协作）
static volatile uint8_t g_in_limit_escape = 0;     // 逃逸阶段屏蔽限位处理
static volatile uint8_t g_motion_active   = 0;     // 当前是否在执行步进
static volatile motor_dir_t g_current_dir = shang; // 当前运动方向
static volatile uint8_t g_abort_now       = 0;     // 请求立即终止当前运动（由ISR置位）
static volatile uint8_t g_escape_pending  = 0;     // ISR提出的逃逸请求
static volatile motor_dir_t g_escape_dir  = xia;   // 逃逸方向
// 统一速度（普通与逃逸共用，可通过函数调整）
static volatile uint16_t g_speed_us = NORMAL_SPEED_US;
// 防抖：记录是否已等待释放（按下->释放）
static volatile uint8_t g_exti0_wait_release = 0;   // PE0 已按下，等待释放
static volatile uint8_t g_exti1_wait_release = 0;   // PE1 已按下，等待释放
// 限位触发标志（用于测试，1=有限位被触发）
static volatile uint8_t g_limit_triggered = 0;

// 快速切换 EXTI 触发边沿（不使用较重的 EXTI_Init）
static inline void EXTI0_SetFalling(void){ EXTI->RTSR &= ~EXTI_Line0; EXTI->FTSR |= EXTI_Line0; }
static inline void EXTI0_SetRising(void) { EXTI->FTSR &= ~EXTI_Line0; EXTI->RTSR |= EXTI_Line0; }
static inline void EXTI1_SetFalling(void){ EXTI->RTSR &= ~EXTI_Line1; EXTI->FTSR |= EXTI_Line1; }
static inline void EXTI1_SetRising(void) { EXTI->FTSR &= ~EXTI_Line1; EXTI->RTSR |= EXTI_Line1; }

/**
  * 函    数：步进电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void StepperMotor_Init(void)
{
    Motor_GPIO_Init();
    Motor_EXTI_Init();
    Motor_Enable(1); // 低电平使能
}

/**
  * 函    数：GPIO初始化
  * 参    数：无
  * 返 回 值：无
  */
static void Motor_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 输出：PA8-PUL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = PUL_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 输出：PB0-DIR, PB1-ENA
    GPIO_InitStructure.GPIO_Pin   = DIR_PIN | ENA_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 输入：PE0/PE1 限位，上拉输入，低电平触发
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = BOTTOM_LIMIT_PIN | TOP_LIMIT_PIN;
    GPIO_Init(LIMIT_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, PUL_PIN);
    GPIO_ResetBits(GPIOB, DIR_PIN);
    GPIO_SetBits(GPIOB, ENA_PIN);   // 默认拉高=禁用
}

/**
  * 函    数：EXTI初始化（限位开关）
  * 参    数：无
  * 返 回 值：无
  */
static void Motor_EXTI_Init(void)
{
    // 将 PE0 -> EXTI0, PE1 -> EXTI1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);

    EXTI_InitTypeDef EXTI_InitStructure;

    // EXTI0 - 底端限位（下降沿）
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // EXTI1 - 顶端限位（下降沿）
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 明确设置触发边沿寄存器，初始为下降沿
    EXTI0_SetFalling();
    EXTI1_SetFalling();

    NVIC_InitTypeDef NVIC_InitStructure;
    // EXTI0 中断
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // EXTI1 中断
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * 函    数：电机使能控制
  * 参    数：enable 1-使能（低电平），0-禁用（高电平）
  * 返 回 值：无
  */
static void Motor_Enable(uint8_t enable)
{
    if(enable) GPIO_ResetBits(GPIOB, ENA_PIN); // 低电平使能
    else       GPIO_SetBits(GPIOB, ENA_PIN);   // 高电平禁用
}

/**
  * 函    数：设置方向为上行
  * 参    数：无
  * 返 回 值：无
  */
static void Motor_SetDirection_Shang(void)
{
    if(DIR_SHANG_LEVEL) GPIO_SetBits(GPIOB, DIR_PIN);
    else                GPIO_ResetBits(GPIOB, DIR_PIN);
}

/**
  * 函    数：设置方向为下行
  * 参    数：无
  * 返 回 值：无
  */
static void Motor_SetDirection_Xia(void)
{
    if(DIR_SHANG_LEVEL) GPIO_ResetBits(GPIOB, DIR_PIN);
    else                GPIO_SetBits(GPIOB, DIR_PIN);
}

/**
  * 函    数：输出指定步数的脉冲
  * 参    数：steps 步数
  *          delay_us 每个高/低电平的延时（微秒）
  * 返 回 值：无
  */
static void Motor_PulseSteps(uint32_t steps, uint32_t delay_us)
{
    for(uint32_t i = 0; i < steps; i++) {
        GPIO_SetBits(GPIOA, PUL_PIN);
        Delay_us(delay_us);
        GPIO_ResetBits(GPIOA, PUL_PIN);
        Delay_us(delay_us);
    }
}

/**
  * 函    数：简单梯形前沿加速脉冲输出
  * 参    数：steps 步数
  * 返 回 值：无
  */
static void Motor_PulseSteps_Sync(uint32_t steps)
{
    if(steps == 0) return;
    uint16_t target = g_speed_us;
    uint16_t slower_us = (target < 200 ? (uint16_t)(target + 120) : (uint16_t)(target + 80));
    if(slower_us > 1000) slower_us = 1000; // 安全上限
    uint32_t accel_steps = steps / 5; // 前20%做加速
    if(accel_steps < 50) accel_steps = (steps > 50 ? 50 : steps/2 + 1);
    for(uint32_t i=0;i<steps;i++){
        uint16_t use_delay = target;
        if(i < accel_steps){
            // 线性插值
            use_delay = (uint16_t)(slower_us - ( (slower_us - target) * (int32_t)i / (int32_t)accel_steps ));
        }
        if(g_abort_now && !g_in_limit_escape) {
            // 被限位打断，提前结束
            break;
        }
        GPIO_SetBits(GPIOA, PUL_PIN);
        Delay_us(use_delay);
        GPIO_ResetBits(GPIOA, PUL_PIN);
        Delay_us(use_delay);
    }
}

/**
  * 函    数：电机移动控制
  * 参    数：direction 方向（shang或xia）
  *          distance_mm 移动距离（毫米）
  * 返 回 值：无
  */
void Motor_Move(motor_dir_t direction, uint32_t distance_mm)
{
    if(distance_mm == 0) return;

    // 设置方向
    if(direction == shang) {
        Motor_SetDirection_Shang();
    } else if(direction == xia) {
        Motor_SetDirection_Xia();
    } else {
        return; // 非法方向
    }

    // 计算步数
    uint32_t steps = distance_mm * (uint32_t)STEPS_PER_MM;

    // 设置运行状态
    g_current_dir = direction;
    g_abort_now = 0;

    if(g_in_limit_escape) {
        // 逃逸阶段：屏蔽限位干预，直接输出脉冲
        Motor_PulseSteps_Sync(steps);
        return;
    }

    g_motion_active = 1;
    for(uint32_t i = 0; i < steps; i++) {
        if(g_abort_now) break; // ISR 请求打断
        GPIO_SetBits(GPIOA, PUL_PIN);
        Delay_us(g_speed_us);
        GPIO_ResetBits(GPIOA, PUL_PIN);
        Delay_us(g_speed_us);
    }
    g_motion_active = 0;

    // 若被限位中断请求打断，则执行反向逃逸 30mm
    if(g_abort_now && g_escape_pending && !g_in_limit_escape) {
        motor_dir_t esc = g_escape_dir;
        g_abort_now = 0;
        g_escape_pending = 0;
        g_in_limit_escape = 1;

        // 执行反向方向逃逸
        if(esc == shang) Motor_SetDirection_Shang(); else Motor_SetDirection_Xia();
        uint32_t esc_steps = LIMIT_ESCAPE_MM * (uint32_t)STEPS_PER_MM;
        Motor_PulseSteps_Sync(esc_steps);

        g_in_limit_escape = 0;
    }
}

/**
  * 函    数：设置运行速度
  * 参    数：us 每个脉冲高/低电平延时（微秒），范围40-1000
  * 返 回 值：无
  */
void Motor_SetSpeedUs(uint16_t us)
{
    if(us < 40) us = 40;       // 过快易失步
    if(us > 1000) us = 1000;   // 过慢无意义
    g_speed_us = us;
}

/**
  * 函    数：获取电机运行状态
  * 参    数：无
  * 返 回 值：1-正在运行，0-空闲
  */
uint8_t Motor_IsRunning(void)
{
    return g_motion_active;
}

/**
  * 函    数：获取限位触发状态并清除标志
  * 参    数：无
  * 返 回 值：1-有限位被触发，0-无触发
  * 说    明：读取后会自动清除标志
  */
uint8_t Motor_GetLimitTriggered(void)
{
    uint8_t triggered = g_limit_triggered;
    g_limit_triggered = 0;  // 读取后清除
    return triggered;
}

/**
  * 函    数：EXTI0中断服务函数（底端限位 PE0）
  * 参    数：无
  * 返 回 值：无
  */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // 防抖：按下->等待释放（切到上升沿），释放->恢复下降沿
        if(g_exti0_wait_release) {
            // 等待释放：只有当电平恢复为高（未触发）时才退出等待并切回下降沿
            if(GPIO_ReadInputDataBit(LIMIT_GPIO_PORT, BOTTOM_LIMIT_PIN) != 0) {
                g_exti0_wait_release = 0;
                EXTI0_SetFalling();
            }
        } else {
            // 未在等待释放，检测按下（低电平）
            if(GPIO_ReadInputDataBit(LIMIT_GPIO_PORT, BOTTOM_LIMIT_PIN) == 0) {
                g_limit_triggered = 1;  // 设置触发标志（用于LED测试）
                if(g_motion_active && (g_current_dir == xia) && !g_in_limit_escape) {
                    g_abort_now = 1;
                    g_escape_pending = 1;
                    g_escape_dir = shang; // 反向上行
                }
                // 切换到上升沿，进入等待释放
                EXTI0_SetRising();
                g_exti0_wait_release = 1;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line0); // 清挂起
    }
}

/**
  * 函    数：EXTI1中断服务函数（顶端限位 PE1）
  * 参    数：无
  * 返 回 值：无
  */
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        // 防抖：按下->等待释放（切到上升沿），释放->恢复下降沿
        if(g_exti1_wait_release) {
            if(GPIO_ReadInputDataBit(LIMIT_GPIO_PORT, TOP_LIMIT_PIN) != 0) {
                g_exti1_wait_release = 0;
                EXTI1_SetFalling();
            }
        } else {
            if(GPIO_ReadInputDataBit(LIMIT_GPIO_PORT, TOP_LIMIT_PIN) == 0) {
                g_limit_triggered = 1;  // 设置触发标志（用于LED测试）
                if(g_motion_active && (g_current_dir == shang) && !g_in_limit_escape) {
                    g_abort_now = 1;
                    g_escape_pending = 1;
                    g_escape_dir = xia; // 反向下行
                }
                EXTI1_SetRising();
                g_exti1_wait_release = 1;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line1); // 清挂起
    }
}
