#include "stm32f10x.h"                  // Device header
#include "ADC.h"
#include "Delay.h"                      // 延时函数

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

/**
  * 函    数：ADC初始化 (液位检测 - PA0引脚)
  * 参    数：无
  * 返 回 值：无
  * 说    明：将PA0配置为ADC输入，用于液位检测，使用直接寄存器操作提高效率
  */
void Adc_Init(void)
{    
    // 先初始化IO口
    RCC->APB2ENR |= 1<<2;    // 使能PORTA时钟 
    GPIOA->CRL &= 0XFFFFFFF0; // PA0 analog输入模式 (清除低4位)
    // 通道0配置			 
    RCC->APB2ENR |= 1<<9;    // ADC1时钟使能	  
    RCC->APB2RSTR |= 1<<9;   // ADC1复位
    RCC->APB2RSTR &= ~(1<<9); // 复位结束	    
    RCC->CFGR &= ~(3<<14);   // 分频因子清零	
    // SYSCLK/DIV6=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
    // 否则将导致ADC准确度下降! 
    RCC->CFGR |= 2<<14;      	 
    ADC1->CR1 &= 0XF0FFFF;   // 工作模式清零
    ADC1->CR1 |= 0<<16;      // 独立工作模式  
    ADC1->CR1 &= ~(1<<8);    // 非扫描模式	  
    ADC1->CR2 &= ~(1<<1);    // 单次转换模式
    ADC1->CR2 &= ~(7<<17);	   
    ADC1->CR2 |= 7<<17;	   // 软件控制转换  
    ADC1->CR2 |= 1<<20;      // 使用外部触发(SWSTART)!!!	必须使用一个事件来触发
    ADC1->CR2 &= ~(1<<11);   // 右对齐	 

    ADC1->SQR1 &= ~(0XF<<20);
    ADC1->SQR1 |= 0<<20;     // 1个转换在规则序列中 也就是只转换规则序列1 			   
    // 设置通道0的采样时间
    ADC1->SMPR2 &= ~(7<<0);  // 通道0采样时间清零	  
    ADC1->SMPR2 |= 7<<0;     // 通道0  239.5周期,提高采样时间可以提高精确度	 

    ADC1->CR2 |= 1<<0;	   // 开启AD转换器	 
    ADC1->CR2 |= 1<<3;       // 使能复位校准  
    while(ADC1->CR2 & 1<<3); // 等待校准结束 			 
    // 复位校准结束后即可设置硬件开始进行校准的寄存器开始校准 		 
    ADC1->CR2 |= 1<<2;        // 开启AD校准	   
    while(ADC1->CR2 & 1<<2);  // 等待校准结束
    // 复位过程结束后就可以开始校准了，校准结束时，硬件清零  
}

/**
  * 函    数：获取ADC值
  * 参    数：ch ADC通道号 (0表示PA0)
  * 返 回 值：ADC转换结果 (0-4095)
  */
uint16_t Get_Adc(uint8_t ch)   
{
    // 设置转换序列	  		 
    ADC1->SQR3 &= 0XFFFFFFE0; // 规则序列1 通道ch
    ADC1->SQR3 |= ch;		  			    
    ADC1->CR2 |= 1<<22;       // 启动规则转换通道 
    while(!(ADC1->SR & 1<<1)); // 等待转换结束	 	   
    return ADC1->DR;		   // 返回adc值	
}

/**
  * 函    数：获取ADC平均值
  * 参    数：ch ADC通道号, times 采样次数
  * 返 回 值：平均值 (0-4095)
  */
uint16_t Get_Adc_Average(uint8_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;
    
    for(t = 0; t < times; t++)
    {
        temp_val += Get_Adc(ch);
        Delay_ms(5);  // 使用项目中的延时函数，稍微增加延时确保采样稳定
    }
    
    return temp_val / times;
}

/**
  * 函    数：将ADC值转换为液位百分比
  * 参    数：adc_value ADC采样值
  * 返 回 值：液位百分比 (0-100)
  * 说    明：根据实际液位传感器特性调整阈值，参考程序文件夹中的标准代码
  */
uint16_t Get_LiquidLevel_Percent(uint16_t adc_value)
{
    uint16_t percent;
    
    // 根据ADC值判断液位百分比 (参考程序文件夹中的标准阈值)
    if(adc_value >= 1100)        // 满液位
    {
        percent = 100;
    }
    else if(adc_value >= 1000)   
    {
        percent = 90;
    }
    else if(adc_value >= 700)   
    {
        percent = 80;
    }
    else if(adc_value >= 540)   
    {
        percent = 70;
    }
    else if(adc_value >= 400)   
    {
        percent = 60;
    }
    else if(adc_value >= 370)   
    {
        percent = 50;
    }
    else if(adc_value >= 330)   
    {
        percent = 40;
    }
    else if(adc_value >= 307)    
    {
        percent = 30;
    }
    else if(adc_value >= 260)    
    {
        percent = 20;
    }
    else if(adc_value >= 194)    
    {
        percent = 10;
    }
    else                         // 空液位
    {
        percent = 0;
    }
    
    return percent;
}