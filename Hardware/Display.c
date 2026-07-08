#include "stm32f10x.h"
#include "Display.h"
#include "tftlcd.h"
#include "Delay.h"
#include <stdio.h>
#include <string.h>

/*
 * Display 模块 —— 森林防火综合控制系统看板
 *
 * 屏幕规格：320 × 480（竖屏，HX8357DN）
 *
 * 实时画面布局：
 *   y =   0 ~  39   标题栏        "Forest Fire System"
 *   y =  50 ~ 150   六舵机区      S1~S6 角度（两行三列）
 *   y = 160 ~ 240   传感器区      Temp / Humi / Yewei
 *   y = 250 ~ 380   模式/继电器   Auto / Relay1 / Relay2
 */

/* 颜色快捷 */
#define BG_COLOR        WHITE
#define TITLE_BG        DARKBLUE
#define TITLE_FG        WHITE
#define LABEL_FG        BLACK
#define VALUE_FG        BLUE
#define ACCENT_FG       RED
#define OK_FG           GREEN
#define OFF_FG          GRAY
#define BORDER_FG       GRAY

/* 上一次显示的数值缓存 —— 仅在变化时刷新 */
static float    s_last_servo[6] = {-999, -999, -999, -999, -999, -999};
static float    s_last_temp = -999, s_last_humi = -999;
static int16_t  s_last_yewei = -1;
static uint8_t  s_last_relay1 = 0xFF, s_last_relay2 = 0xFF;
static uint8_t  s_last_auto   = 0xFF;
static uint8_t  s_ui_drawn    = 0;

/* 触摸按钮区几何定义（两个按钮左右排列） */
#define BTN1_X0   20
#define BTN1_X1   150
#define BTN2_X0   170
#define BTN2_X1   300
#define BTN_Y0    415
#define BTN_Y1    465

/* 按钮触摸状态：按下位置落在哪个按钮上（0=无，1=按钮1，2=按钮2） */
static uint8_t  s_btn_pressed_idx = 0;   /* 本次按下命中的按钮，0=无 */
static uint8_t  s_last_touch_down = 0;   /* 上一轮的按压状态（用于边沿检测） */
/* 按钮外观缓存：0=未画,1=常态,2=按下高亮；relay 状态缓存用于文字刷新 */
static uint8_t  s_btn1_look = 0xFF;
static uint8_t  s_btn2_look = 0xFF;
static uint8_t  s_btn_last_relay1 = 0xFF;
static uint8_t  s_btn_last_relay2 = 0xFF;

/* 工具：在矩形区域先用背景色清除再画字符串 */
static void draw_text(u16 x, u16 y, u16 w, u16 h, const char *str, u8 size, u16 color)
{
	LCD_Fill(x, y, x + w - 1, y + h - 1, BG_COLOR);
	FRONT_COLOR = color;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(x, y, w, h, size, (u8 *)str);
}

static void draw_value_int(u16 x, u16 y, u16 w, u16 h, int value, u8 size, u16 color)
{
	char buf[16];
	sprintf(buf, "%d", value);
	draw_text(x, y, w, h, buf, size, color);
}

static void draw_value_float1(u16 x, u16 y, u16 w, u16 h, float value, u8 size, u16 color)
{
	char buf[16];
	sprintf(buf, "%.1f", value);
	draw_text(x, y, w, h, buf, size, color);
}

/*-----------------------------------------------------------------------------
 * 开机动画
 *---------------------------------------------------------------------------*/
void Display_BootAnimation(void)
{
	u16 i;

	LCD_Clear(BLACK);

	FRONT_COLOR = WHITE;
	BACK_COLOR  = BLACK;
	LCD_ShowString(110, 100, 200, 30, 24, (u8 *)"STM32");
	Delay_ms(150);

	FRONT_COLOR = CYAN;
	LCD_ShowString(40, 140, 260, 30, 24, (u8 *)"Forest Fire System");
	Delay_ms(300);

	FRONT_COLOR = WHITE;
	LCD_ShowString(120, 200, 200, 16, 16, (u8 *)"LOADING...");

	FRONT_COLOR = WHITE;
	LCD_DrawRectangle(40, 230, 280, 254);

	for (i = 0; i <= 100; i += 5)
	{
		u16 fillEnd = 42 + (236 * i / 100);
		LCD_Fill(42, 232, fillEnd, 252, GREEN);

		{
			char buf[8];
			sprintf(buf, "%3d%%", i);
			FRONT_COLOR = YELLOW;
			BACK_COLOR  = BLACK;
			LCD_ShowString(135, 270, 60, 16, 16, (u8 *)buf);
		}
		Delay_ms(15);
	}
	Delay_ms(250);

	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR  = BLACK;
	LCD_ShowString(80, 60, 200, 24, 24, (u8 *)"System Init");

	LCD_ShowString(40, 120, 200, 16, 16, (u8 *)"Servo  ......");
	Delay_ms(120);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 120, 60, 16, 16, (u8 *)"OK");

	FRONT_COLOR = WHITE;
	LCD_ShowString(40, 150, 200, 16, 16, (u8 *)"Sensor ......");
	Delay_ms(120);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 150, 60, 16, 16, (u8 *)"OK");

	FRONT_COLOR = WHITE;
	LCD_ShowString(40, 180, 200, 16, 16, (u8 *)"Stepper......");
	Delay_ms(120);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 180, 60, 16, 16, (u8 *)"OK");

	FRONT_COLOR = WHITE;
	LCD_ShowString(40, 210, 200, 16, 16, (u8 *)"Comm   ......");
	Delay_ms(120);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 210, 60, 16, 16, (u8 *)"OK");

	Delay_ms(250);

	for (i = 0; i < 3; i++)
	{
		FRONT_COLOR = YELLOW;
		BACK_COLOR  = BLACK;
		LCD_ShowString(110, 280, 200, 24, 24, (u8 *)"READY!");
		Delay_ms(180);
		LCD_Fill(110, 280, 250, 310, BLACK);
		Delay_ms(100);
	}

	s_ui_drawn = 0;
}

/*-----------------------------------------------------------------------------
 * 静态界面（标题、表头、分隔线）
 *---------------------------------------------------------------------------*/
void Display_DrawStaticUI(void)
{
	u16 W = tftlcd_data.width;

	LCD_Clear(BG_COLOR);

	/* 标题栏 */
	LCD_Fill(0, 0, W - 1, 39, TITLE_BG);
	FRONT_COLOR = TITLE_FG;
	BACK_COLOR  = TITLE_BG;
	LCD_ShowString(20, 8, W - 40, 24, 24, (u8 *)"Forest Fire System");

	/* 区块边框 */
	FRONT_COLOR = BORDER_FG;
	LCD_DrawRectangle(5,  50, W - 6, 150);   /* 六舵机区 */
	LCD_DrawRectangle(5, 160, W - 6, 240);   /* 传感器区 */
	LCD_DrawRectangle(5, 250, W - 6, 380);   /* 模式/继电器区 */
	LCD_DrawRectangle(5, 390, W - 6, 475);   /* 触摸测试区 */

	/* === 触摸按钮区表头 === */
	FRONT_COLOR = LABEL_FG;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(15, 398, 250, 16, 16, (u8 *)"Touch Buttons");
	/* 按钮外观在 Display_ScanButtons 首次调用时绘制（依赖当前继电器状态） */

	FRONT_COLOR = LABEL_FG;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(15, 58, 200, 16, 16, (u8 *)"Servo Angles");

	/* 第一行：S1 S2 S3 */
	LCD_ShowString(15,  90, 30, 16, 16, (u8 *)"S1:");
	LCD_ShowString(115, 90, 30, 16, 16, (u8 *)"S2:");
	LCD_ShowString(215, 90, 30, 16, 16, (u8 *)"S3:");
	/* 第二行：S4 S5 S6 */
	LCD_ShowString(15,  120, 30, 16, 16, (u8 *)"S4:");
	LCD_ShowString(115, 120, 30, 16, 16, (u8 *)"S5:");
	LCD_ShowString(215, 120, 30, 16, 16, (u8 *)"S6:");

	/* === 传感器区表头 === */
	LCD_ShowString(15, 168, 200, 16, 16, (u8 *)"Temp / Humi / Level");
	LCD_ShowString(15, 195, 30,  16, 16, (u8 *)"T:");
	LCD_ShowString(115, 195, 30, 16, 16, (u8 *)"H:");
	LCD_ShowString(215, 195, 50, 16, 16, (u8 *)"YW:");

	/* === 模式区表头 === */
	LCD_ShowString(15, 258, 200, 16, 16, (u8 *)"Mode & Output");
	LCD_ShowString(15, 290, 60,  16, 16, (u8 *)"Auto:");
	LCD_ShowString(15, 320, 80,  16, 16, (u8 *)"Relay1:");
	LCD_ShowString(165, 320, 80, 16, 16, (u8 *)"Relay2:");

	/* 重置缓存，强制下次 UpdateData 全部刷一遍 */
	{
		u8 i;
		for (i = 0; i < 6; i++) s_last_servo[i] = -999;
	}
	s_last_temp = s_last_humi = -999;
	s_last_yewei = -1;
	s_last_relay1 = s_last_relay2 = 0xFF;
	s_last_auto = 0xFF;

	s_btn_pressed_idx = 0;
	s_last_touch_down = 0;
	s_btn1_look = s_btn2_look = 0xFF;   /* 强制下次重绘按钮 */
	s_btn_last_relay1 = s_btn_last_relay2 = 0xFF;

	s_ui_drawn = 1;
}

/*-----------------------------------------------------------------------------
 * 实时数据更新（差分刷新）
 *---------------------------------------------------------------------------*/
void Display_UpdateData(float servo1, float servo2, float servo3,
                        float servo4, float servo5, float servo6,
                        float temp, float humi,
                        uint16_t yewei,
                        uint8_t relay1, uint8_t relay2,
                        uint8_t auto_mode)
{
	float servo[6];
	const u16 col_x[3] = {45, 145, 245};
	const u16 row_y[2] = {90, 120};
	u8 i;

	if (!s_ui_drawn)
	{
		Display_DrawStaticUI();
	}

	servo[0] = servo1; servo[1] = servo2; servo[2] = servo3;
	servo[3] = servo4; servo[4] = servo5; servo[5] = servo6;

	/* === 六舵机角度 === */
	for (i = 0; i < 6; i++)
	{
		if (servo[i] != s_last_servo[i])
		{
			draw_value_int(col_x[i % 3], row_y[i / 3], 60, 16,
			               (int)servo[i], 16, VALUE_FG);
			s_last_servo[i] = servo[i];
		}
	}

	/* === 温度 === */
	if (temp != s_last_temp)
	{
		draw_value_float1(45, 195, 60, 16, temp, 16, ACCENT_FG);
		s_last_temp = temp;
	}

	/* === 湿度 === */
	if (humi != s_last_humi)
	{
		draw_value_float1(145, 195, 60, 16, humi, 16, VALUE_FG);
		s_last_humi = humi;
	}

	/* === 液位（百分比） === */
	if ((int16_t)yewei != s_last_yewei)
	{
		char buf[8];
		sprintf(buf, "%d%%", yewei);
		draw_text(255, 195, 50, 16, buf, 16, VALUE_FG);
		s_last_yewei = (int16_t)yewei;
	}

	/* === 模式：auto === */
	if (auto_mode != s_last_auto)
	{
		if (auto_mode)
			draw_text(75, 290, 80, 16, "ON ", 16, OK_FG);
		else
			draw_text(75, 290, 80, 16, "OFF", 16, OFF_FG);
		s_last_auto = auto_mode;
	}

	/* === 继电器1 === */
	if (relay1 != s_last_relay1)
	{
		if (relay1)
			draw_text(95, 320, 60, 16, "ON ", 16, OK_FG);
		else
			draw_text(95, 320, 60, 16, "OFF", 16, OFF_FG);
		s_last_relay1 = relay1;
	}

	/* === 继电器2 === */
	if (relay2 != s_last_relay2)
	{
		if (relay2)
			draw_text(245, 320, 60, 16, "ON ", 16, OK_FG);
		else
			draw_text(245, 320, 60, 16, "OFF", 16, OFF_FG);
		s_last_relay2 = relay2;
	}
}

/*-----------------------------------------------------------------------------
 * 触摸按钮区（替代原滑动条）
 *   - 两个按钮：Relay1 / Relay2 开关
 *   - 采用“按下→抬起”边沿触发，松手瞬间上报一次点击
 *   - 按钮外观随继电器状态高亮（ON=绿，OFF=灰），差分刷新
 *---------------------------------------------------------------------------*/

/* 判断坐标是否落在某按钮矩形内（1=按钮1，2=按钮2，0=区外） */
static uint8_t btn_hit_test(int16_t x, int16_t y)
{
	if (y < BTN_Y0 || y > BTN_Y1) return 0;
	if (x >= BTN1_X0 && x <= BTN1_X1) return 1;
	if (x >= BTN2_X0 && x <= BTN2_X1) return 2;
	return 0;
}

/* 绘制单个按钮：idx=1/2；on=继电器状态；pressed_look=是否按下高亮 */
static void draw_button(uint8_t idx, uint8_t on, uint8_t pressed_look)
{
	u16 x0 = (idx == 1) ? BTN1_X0 : BTN2_X0;
	u16 x1 = (idx == 1) ? BTN1_X1 : BTN2_X1;
	u16 fill;
	const char *label = (idx == 1) ? "Relay1" : "Relay2";
	const char *state = on ? "ON " : "OFF";
	u16 tx;

	/* 按下时用深色反馈，否则按继电器状态着色 */
	if (pressed_look) fill = DARKBLUE;
	else              fill = on ? OK_FG : OFF_FG;

	LCD_Fill(x0, BTN_Y0, x1, BTN_Y1, fill);
	FRONT_COLOR = BORDER_FG;
	LCD_DrawRectangle(x0, BTN_Y0, x1, BTN_Y1);

	/* 按钮文字：标题 + 状态，白字 */
	FRONT_COLOR = WHITE;
	BACK_COLOR  = fill;
	tx = x0 + ((x1 - x0) - 6 * 8) / 2;   /* 6字符*8px(16号半宽) 居中 */
	LCD_ShowString(tx, BTN_Y0 + 8,  x1 - x0, 16, 16, (u8 *)label);
	tx = x0 + ((x1 - x0) - 3 * 8) / 2;
	LCD_ShowString(tx, BTN_Y0 + 28, x1 - x0, 16, 16, (u8 *)state);
}

DisplayButton_t Display_ScanButtons(int16_t touch_x, int16_t touch_y, uint8_t pressed,
                                    uint8_t relay1, uint8_t relay2)
{
	DisplayButton_t clicked = DISPLAY_BTN_NONE;
	uint8_t look1, look2;

	if (!s_ui_drawn)
	{
		Display_DrawStaticUI();
	}

	/* === 边沿检测：抬起瞬间判定点击 === */
	if (pressed && !s_last_touch_down)
	{
		/* 刚按下：记录命中的按钮 */
		s_btn_pressed_idx = btn_hit_test(touch_x, touch_y);
	}
	else if (!pressed && s_last_touch_down)
	{
		/* 刚抬起：若与按下时是同一按钮上，则确认为一次点击 */
		if (s_btn_pressed_idx == 1)      clicked = DISPLAY_BTN_RELAY1;
		else if (s_btn_pressed_idx == 2) clicked = DISPLAY_BTN_RELAY2;
		s_btn_pressed_idx = 0;
	}
	s_last_touch_down = pressed;

	/* === 差分刷新按钮外观 === */
	/* 期望外观：2=当前正被按住高亮，1=常态 */
	look1 = (pressed && s_btn_pressed_idx == 1) ? 2 : 1;
	look2 = (pressed && s_btn_pressed_idx == 2) ? 2 : 1;

	if (look1 != s_btn1_look || relay1 != s_btn_last_relay1)
	{
		draw_button(1, relay1, (look1 == 2));
		s_btn1_look = look1;
		s_btn_last_relay1 = relay1;
	}
	if (look2 != s_btn2_look || relay2 != s_btn_last_relay2)
	{
		draw_button(2, relay2, (look2 == 2));
		s_btn2_look = look2;
		s_btn_last_relay2 = relay2;
	}

	return clicked;
}
