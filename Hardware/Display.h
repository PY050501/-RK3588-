#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "stm32f10x.h"

/*
 * Display 模块（TFTLCD 版） —— 森林防火综合控制看板
 *  - 屏幕：3.5寸 320x480 彩屏（HX8357DN，竖屏 320 宽 × 480 高）
 *  - 由 main.c 调用：开机动画 + 实时数据刷新
 *  - 字段：六舵机 + 温湿度 + 液位 + 继电器 + 模式
 */

/* 开机动画 */
void Display_BootAnimation(void);

/* 静态界面：清屏 + 绘制标题栏 / 区块边框 / 表头（首次进入时自动调用） */
void Display_DrawStaticUI(void);

/* 实时数据更新（差分刷新，仅变化字段重绘） */
void Display_UpdateData(float servo1, float servo2, float servo3,
                        float servo4, float servo5, float servo6,
                        float temp, float humi,
                        uint16_t yewei,
                        uint8_t relay1, uint8_t relay2,
                        uint8_t auto_mode);

/* 触摸按钮区 —— y=390~475
 *   区内放置两个按钮：Relay1 / Relay2 开关
 *   Display_ScanButtons 传入当前触摸坐标与按压状态，返回“本次被点击”的按钮：
 *     DISPLAY_BTN_NONE   无点击
 *     DISPLAY_BTN_RELAY1 点击了继电器1按钮
 *     DISPLAY_BTN_RELAY2 点击了继电器2按钮
 *   采用“按下→抬起”边沿触发，一次点击只上报一次，避免长按重复触发。
 */
#define DISPLAY_BTN_AREA_Y0   390
#define DISPLAY_BTN_AREA_Y1   475

typedef enum
{
	DISPLAY_BTN_NONE = 0,
	DISPLAY_BTN_RELAY1,
	DISPLAY_BTN_RELAY2
} DisplayButton_t;

/* 扫描按钮区，返回本次点击到的按钮（松手瞬间上报一次）。
 * relay1/relay2 为当前继电器状态，用于按钮高亮显示。 */
DisplayButton_t Display_ScanButtons(int16_t touch_x, int16_t touch_y, uint8_t pressed,
                                    uint8_t relay1, uint8_t relay2);

#endif
