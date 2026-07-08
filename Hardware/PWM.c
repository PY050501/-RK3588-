#include "stm32f10x.h"                  // Device header

/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void PWM_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//开启TIM2的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟（用于舵机5）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//开启TIM4的时钟（用于舵机4）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);			//开启TIM8的时钟（用于舵机6，PC6）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//开启GPIOB的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);			//开启GPIOC的时钟（舵机6：PC6）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			//开启AFIO时钟

	/* 重映射顺序很关键：必须先做普通 remap，最后做 SWJ 配置。
	 * 因为 SPL 的 GPIO_PinRemapConfig 在处理普通 remap 时会对 MAPR 执行
	 * (tmpreg |= ~DBGAFR_SWJCFG_MASK)，把 SWJ_CFG 位域写成全1（禁用SWD/JTAG）。
	 * 若先配 SWJ、再做 TIM3 remap，SWD 会被 TIM3 那次调用重新冲掉。 */

	/* TIM3部分重映射：CH2 由 PA7 改到 PB5（舵机5引脚变更）。
	 * 注意：部分重映射同时把 TIM3 的 CH1/CH3/CH4 指向 PB4/PB0/PB1，
	 *       但本工程只使能 TIM3_CH2 输出，未使能 CH1/CH3/CH4，
	 *       故 PB0/PB1（丝杆 DIR/ENA）仍作普通GPIO使用，不受影响。 */
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	/* SWJ 配置放最后：释放JTAG引脚(PA15/PB3/PB4)，保留SWD */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/*GPIO初始化 - GPIOA (舵机1:PA1, 舵机2:PA2, 舵机3:PA3)*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIO初始化 - GPIOB (舵机4:PB6, 舵机5:PB5-TIM3_CH2重映射)
	 * 注意：舵机6 原用 PB7(TIM4_CH2)，因 PB7=FSMC_NADV 被彩屏 FSMC 抢占无法输出PWM，
	 *       已迁移到 PC6(TIM8_CH1)，见下方 GPIOC 初始化。*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*GPIO初始化 - GPIOC (舵机6:PC6-TIM8_CH1)*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	/*配置时钟源 - TIM2*/
	TIM_InternalClockConfig(TIM2);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化 - TIM2*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元
	
	/*配置时钟源 - TIM3*/
	TIM_InternalClockConfig(TIM3);		//选择TIM3为内部时钟
	
	/*时基单元初始化 - TIM3*/
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
	
	/*配置时钟源 - TIM4*/
	TIM_InternalClockConfig(TIM4);		//选择TIM4为内部时钟
	
	/*时基单元初始化 - TIM4*/
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM4的时基单元

	/*配置时钟源 - TIM8（舵机6，PC6）*/
	TIM_InternalClockConfig(TIM8);		//选择TIM8为内部时钟
	/*时基单元初始化 - TIM8。TIM8在APB2上，72MHz时钟，与APB1定时器相同，
	 * 同一时基结构体(PSC=71,ARR=19999)得到50Hz/1us分辨率*/
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);            //配置TIM8的时基单元

	/*输出比较初始化 - TIM2*/
	TIM_OCInitTypeDef TIM_OCInitStructure;							//定义结构体变量
	TIM_OCStructInit(&TIM_OCInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //输出比较模式，选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出使能
	TIM_OCInitStructure.TIM_Pulse = 1500;							//初始的CCR值 (1.5ms对应90度)
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC2Init，配置TIM2的输出比较通道2 (PA1-舵机1)
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC3Init，配置TIM2的输出比较通道3 (PA2-舵机2)
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC4Init，配置TIM2的输出比较通道4 (PA3-舵机3)
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*输出比较初始化 - TIM3*/
	TIM_OCInitStructure.TIM_Pulse = 500;							//初始的CCR值 (0.5ms对应0度)
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC2Init，配置TIM3的输出比较通道2 (PB5-舵机5，TIM3部分重映射)
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/*输出比较初始化 - TIM4*/
	TIM_OCInitStructure.TIM_Pulse = 500;							//初始的CCR值 (0.5ms对应0度)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC1Init，配置TIM4的输出比较通道1 (PB6-舵机4)
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/*输出比较初始化 - TIM8通道1 (PC6-舵机6)*/
	TIM_OCInitStructure.TIM_Pulse = 500;							//初始的CCR值 (0.5ms对应0度)
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);                        //配置TIM8的输出比较通道1 (PC6-舵机6)
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	/*TIM使能*/
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
	TIM_Cmd(TIM3, ENABLE);			//使能TIM3，定时器开始运行
	TIM_Cmd(TIM4, ENABLE);			//使能TIM4，定时器开始运行
	TIM_Cmd(TIM8, ENABLE);			//使能TIM8，定时器开始运行

	/* TIM8是高级定时器，PWM输出必须使能主输出(MOE位)，否则通道无波形。
	 * 普通定时器(TIM2/3/4)无此要求。*/
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	/*TIM ARR预装载使能*/
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);
}

/**
  * 函    数：PWM设置CCR (TIM2通道2)
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);		//设置CCR2的值
}

/**
  * 函    数：PWM设置CCR3
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);		//设置CCR3的值
}

/**
  * 函    数：PWM设置CCR4
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare4(TIM2, Compare);		//设置CCR4的值
}

/**
  * 函    数：PWM设置CCR (TIM4通道1)
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  *           此函数用于控制PB6引脚上的舵机4
  */
void PWM_SetCompare4_TIM4(uint16_t Compare)
{
	TIM_SetCompare1(TIM4, Compare);		//设置TIM4的CCR1的值 (PB6-舵机4)
}

/**
  * 函    数：PWM设置CCR (TIM3通道2)
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  *           此函数用于控制PB5引脚上的舵机5（TIM3_CH2部分重映射）
  */
void PWM_SetCompare2_TIM3(uint16_t Compare)
{
	TIM_SetCompare2(TIM3, Compare);		//设置TIM3的CCR2的值 (PB5-舵机5)
}

/**
  * 函    数：PWM设置CCR (TIM4通道2)
  * 参    数：Compare 要写入的CCR的值，范围：0~2500
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  *           此函数用于控制PC6引脚上的舵机6（TIM8_CH1）
  */
void PWM_SetCompare2_TIM4(uint16_t Compare)
{
	TIM_SetCompare1(TIM8, Compare);		//设置TIM8的CCR1的值 (PC6-舵机6)
}
