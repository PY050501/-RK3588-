#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"                  // Device header

// 类型定义兼容性处理
#ifndef u8
typedef uint8_t u8;
#endif
#ifndef u16
typedef uint16_t u16;
#endif
#ifndef u32
typedef uint32_t u32;
#endif

// ADC通道定义
#define ADC_CH0         0               // 通道0 (PA0)
#define ADC_CH_LIQUID   0               // 液位检测通道 (PA0)
#define ADC_CH1         1               // 通道1 (PA1，备用)

// ADC液位检测相关函数声明
void Adc_Init(void);                                                    // ADC初始化
uint16_t Get_Adc(uint8_t ch);                                           // 获取ADC值
uint16_t Get_Adc_Average(uint8_t ch, uint8_t times);                    // 获取ADC平均值
uint16_t Get_LiquidLevel_Percent(uint16_t adc_value);                   // 将ADC值转换为液位百分比

// 兼容性函数声明 (支持u8/u16类型)
u16 Get_Adc(u8 ch);                                                     // 兼容性函数
u16 Get_Adc_Average(u8 ch, u8 times);                                   // 兼容性函数

#endif
