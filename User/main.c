#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Servo.h"
#include "Serial.h"
#include "bsp_si7021.h"
#include "bsp_i2c.h"
#include "cJSON.h"
#include "StepperMotor.h"               // 步进电机模块
#include "ADC.h"                        // ADC液位检测模块
#include "motor.h"
#include "SysTick.h"                    // 触摸/LCD 用 delay_us/delay_ms
#include "tftlcd.h"                     // TFTLCD 驱动
#include "touch.h"                      // 电阻触摸驱动
#include "Display.h"                    // 看板模块
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 安全的JSON响应发送宏（防止内存泄漏）
#define SEND_JSON_RESPONSE(json_obj) \
	do { \
		if (json_obj != NULL) { \
			char *response_str = cJSON_PrintUnformatted(json_obj); \
			if (response_str != NULL) { \
				Serial_Printf("%s\r\n", response_str); \
				free(response_str); \
			} \
			cJSON_Delete(json_obj); \
		} \
	} while(0)


/**
  * 文件说明：
  *   - 本文件为六舵机PWM控制 + 丝杆步进电机控制 + 液位检测主程序，支持JSON与传统命令双协议。
  *   - 集成ADC液位检测功能，使用PA0引脚进行液位检测。
  *   - 舵机角度全局变量（在Servo.c中定义，这里声明）
  *     extern float Servo1_Angle; // 舵机1角度全局变量
  *     extern float Servo2_Angle; // 舵机2角度全局变量
  *     extern float Servo3_Angle; // 舵机3角度全局变量（上电默认90度）
  *     extern float Servo4_Angle; // 舵机4角度全局变量（PB6引脚）
  *     extern float Servo5_Angle; // 舵机5角度全局变量（PB5引脚）
  *     extern float Servo6_Angle; // 舵机6角度全局变量（PB7引脚）
  *   - 这些变量可被主机直接访问和控制（已在Servo.h声明，这里无需重复声明）。
  *
  * JSON命令格式说明：
  *   1. 设置单个舵机角度：
  *      {"servo1": 90}           - 设置舵机1到90度
  *      {"servo2": 45}           - 设置舵机2到45度
  *      {"servo3": 60}           - 设置舵机3到60度
  *      {"servo4": 60}           - 设置舵机4到60度（PB6引脚）
  *      {"servo5": 90}           - 设置舵机5到90度（PB5引脚）
  *      {"servo6": 45}           - 设置舵机6到45度（PB7引脚）
  *   2. 同时设置多个舵机：
  *      {"servo1": 90, "servo2": 45, "servo3": 60, "servo4": 60, "servo5": 90, "servo6": 45}
  *   3. 使用传统命令：
  *      {"command": "S1G1"}      - 执行舵机1档位1命令
  *      {"command": "ALLHOME"}   - 执行所有舵机归零命令
  *
  *   4. 快捷动作命令：
  *      {"action": "home"}       - 舵机1归零到100度，舵机2归零到50度，舵机3归零到90度，舵机4归零到90度，舵机5-6归零到0度
  *      {"action": "status"}     - 查询当前所有舵机状态
  *   5. 舵机1自动模式切换：
  *      {"auto": 0}         - 启用舵机1自动模式(10秒循环0-180-0度)
  *      {"auto": 1}         - 启用舵机1手动模式(接收指令控制)
  *   6. 步进电机（丝杆）控制：
  *      {"motor": "up", "distance": 100}    - 上升100mm
  *      {"motor": "down", "distance": 50}   - 下降50mm
  *      {"motor": "shang", "distance": 200} - 上升200mm（中文）
  *      {"motor": "xia", "distance": 150}   - 下降150mm（中文）
  *   7. 继电器控制：
  *      {"switch": 1}       - 打开继电器1（PB8）
  *      {"switch": 0}       - 关闭继电器1（PB8）
  *      {"switch1": 1}      - 打开继电器2（PB9）
  *      {"switch1": 0}      - 关闭继电器2（PB9）
  *   8. 无人机座舱步进电机控制：
  *      {"fly": 1}          - 顺时针转1.3圈（fly开）
  *      {"fly": 0}          - 逆时针转1.5圈回起点（fly关）
  *
  * 响应格式：
  *   - 成功时返回确认消息，如：{"servo1_set": 90}, {"servo5_set": 90}
  *   - 错误时返回错误信息，如：{"error": "Unknown command format"}
  *   - 状态查询返回：{"servo1": 90, "servo2": 0, "servo3": 90, "servo4": 0, "servo5": 0, "servo6": 0, "auto": 1, "status": "ok"}
  *   - 自动模式切换：{"auto_set": 0, "description": "Servo1 auto mode enabled"}
  *   - 步进电机响应：{"motor_executed": "up", "distance_mm": 100}
  *   - 继电器响应：{"switch_set": 1, "description": "Relay ON (high level)"}
  *   - 继电器2响应：{"switch1_set": 1, "description": "Relay2 ON (high level)"}
  *   - 液位检测回报：{"yewei": 50}
  *   - fly响应：{"fly_set": 1}
  *
  *
  *  非JSON传统命令说明（直接串口发送字符串）：
  *   - S1G1/S1G2/S1G3/S1G4/S1HOME   控制舵机1档位/归零
  *   - S2G1/S2G2/S2G3/S2G4/S2HOME   控制舵机2档位/归零
  *   - S3G1/S3G2/S3G3/S3G4/S3HOME   控制舵机3档位/归零（限位0-90度，HOME为90度）
  *   - S4G1/S4G2/S4G3/S4G4/S4HOME   控制舵机4档位/归零（PB6引脚）
  *   - BOTH1/BOTH2/BOTH3/BOTH4/BOTHOME   同时控制舵机1和2
  *   - ALL1/ALL2/ALL3/ALL4/ALLHOME      同时控制所有舵机
  *   - 发送如 S1G1 + 回车 即可直接控制，无需JSON封装
  *
  * 舵机1自动旋转功能：
  *   - moshiqiehuan = 0: 舵机1自动模式，10秒内从0-180-0度循环
  *   - moshiqiehuan = 1: 舵机1手动模式，接收指令控制
  *   - JSON切换命令：{"auto": 0} 或 {"auto": 1}
  *
 * 步进电机（丝杆）引脚分配：
 *   - PA8: PUL（脉冲）
 *   - PB0: DIR（方向）
 *   - PB1: ENA（使能，低电平有效）
 *   - PE0: 底端限位（EXTI0，低电平触发）
 *   - PE1: 顶端限位（EXTI1，低电平触发）
  *
  * 液位传感器引脚分配：
  *   - PA0: ADC液位检测（ADC1_CH0）
  *
  * 继电器引脚分配：
  *   - PB8: 继电器1（高电平启动）
  *   - PB9: 继电器2（高电平启动）
  *
  * 无人机座舱步进电机引脚分配（28BYJ-48，ULN2003驱动）：
  *   - PB12: IN1
  *   - PB13: IN2
  *   - PB14: IN3
  *   - PB15: IN4
  */


// 函数声明
void Relay_GPIO_Set(uint8_t state);
void Relay2_GPIO_Set(uint8_t state);
void Yuyin_GPIO_Init(void);
void Yuyin_GPIO_Set(uint8_t state);

// 舵机1自动旋转模式标志位
uint8_t moshiqiehuan = 1;  // 0=自动模式, 1=手动模式
// 舵机1自动模式停止标志位（0=旋转，1=停止），上电默认0
uint8_t servo1stop = 0;

// 继电器高电平启动模式标志位
uint8_t relay_switch = 0; // 0=关闭, 1=高电平启动（PB8）
uint8_t relay_switch1 = 0; // 0=关闭, 1=高电平启动（PB9）

// 语音控制引脚状态标志位（PB10）
uint8_t yuyin_state = 0; // 0=高电平, 1=低电平，默认0（高电平）

// JSON内存分配失败计数器（调试用）
static uint16_t json_alloc_fail_count = 0;
static uint32_t command_counter = 0; // 命令计数器

// 安全发送JSON响应（防止内存泄漏）
void SendJSONResponseSafe(cJSON *response)
{
	if (response == NULL)
	{
		json_alloc_fail_count++;
		return;
	}

	char *response_str = cJSON_PrintUnformatted(response);
	if (response_str != NULL)
	{
		Serial_Printf("%s\r\n", response_str);
		free(response_str);
	}
	else
	{
		json_alloc_fail_count++;
	}
	cJSON_Delete(response);
}

// 详见Servo.h注释
uint8_t ProcessJSONCommand(char* json_str)
{
	cJSON *json = cJSON_Parse(json_str);
	if (json == NULL)
	{
		// JSON解析失败，发送错误响应并立即返回
		cJSON *error_response = cJSON_CreateObject();
		if (error_response != NULL)
		{
			cJSON_AddItemToObject(error_response, "error", cJSON_CreateString("Invalid JSON format"));
			cJSON_AddItemToObject(error_response, "received", cJSON_CreateString(json_str));
		}
		SendJSONResponseSafe(error_response);
		return 0;	// JSON解析失败
	}

	uint8_t processed = 0;

	// 创建统一的响应对象（减少内存分配次数）
	cJSON *unified_response = cJSON_CreateObject();
	uint8_t has_response = 0;

	// 添加命令序号（用于调试和追踪）
	command_counter++;
	if (unified_response != NULL)
	{
		cJSON_AddItemToObject(unified_response, "cmd_id", cJSON_CreateNumber(command_counter));
	}

	// 检查servo1角度设置
	cJSON *servo1_item = cJSON_GetObjectItem(json, "servo1");
	if (servo1_item != NULL && servo1_item->type == cJSON_Number)
	{
		float angle = (float)servo1_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			// 只有在手动模式下才允许设置舵机1角度
			if (moshiqiehuan == 1)
			{
				Servo1_Angle = angle;
				processed = 1;
				has_response = 1;
				// 发送确认消息
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "servo1_set", cJSON_CreateNumber(angle));
				}
			}
			else
			{
				// 自动模式下拒绝手动设置
				processed = 1;
				has_response = 1;
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "error", cJSON_CreateString("Servo1 in auto mode"));
				}
			}
		}
	}

	// 检查servo2角度设置
	cJSON *servo2_item = cJSON_GetObjectItem(json, "servo2");
	if (servo2_item != NULL && servo2_item->type == cJSON_Number)
	{
		float angle = (float)servo2_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
		Servo2_Angle = angle;
		processed = 1;
		has_response = 1;
		if (unified_response != NULL)
		{
			cJSON_AddItemToObject(unified_response, "servo2_set", cJSON_CreateNumber(angle));
		}
	}
}	// 检查servo3角度设置
	cJSON *servo3_item = cJSON_GetObjectItem(json, "servo3");
	if (servo3_item != NULL && servo3_item->type == cJSON_Number)
	{
		float angle = (float)servo3_item->valuedouble;
		if (angle >= 0 && angle <= 180)  // 舵机3取消限位，改为0-180度
		{
			Servo3_Angle = angle;
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "servo3_set", cJSON_CreateNumber(angle));
			}
		}
	}

	// 检查servo4角度设置
	cJSON *servo4_item = cJSON_GetObjectItem(json, "servo4");
	if (servo4_item != NULL && servo4_item->type == cJSON_Number)
	{
		float angle = (float)servo4_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			Servo4_Angle = angle;
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "servo4_set", cJSON_CreateNumber(angle));
			}
		}
	}

	// 检查servo5角度设置
	cJSON *servo5_item = cJSON_GetObjectItem(json, "servo5");
	if (servo5_item != NULL && servo5_item->type == cJSON_Number)
	{
		float angle = (float)servo5_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			Servo5_Angle = angle;
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "servo5_set", cJSON_CreateNumber(angle));
			}
		}
	}

	// 检查servo6角度设置
	cJSON *servo6_item = cJSON_GetObjectItem(json, "servo6");
	if (servo6_item != NULL && servo6_item->type == cJSON_Number)
	{
		float angle = (float)servo6_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			Servo6_Angle = angle;
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "servo6_set", cJSON_CreateNumber(angle));
			}
		}
	}

	// 检查自动模式切换命令（兼容 auto 和 auto_mode 字段）
	cJSON *auto_item = cJSON_GetObjectItem(json, "auto");
	if (auto_item == NULL) {
		auto_item = cJSON_GetObjectItem(json, "auto_mode");
	}
	if (auto_item != NULL && auto_item->type == cJSON_Number)
	{
		int mode = (int)auto_item->valuedouble;
		if (mode == 0 || mode == 1)
		{
			moshiqiehuan = mode;
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "auto_set", cJSON_CreateNumber(mode));
			}
		}
	}

	// 检查servo1stop命令（仅在自动模式下有效）
	cJSON *servo1stop_item = cJSON_GetObjectItem(json, "servo1stop");
	if (servo1stop_item != NULL && servo1stop_item->type == cJSON_Number)
	{
		int stopval = (int)servo1stop_item->valuedouble;
		if (moshiqiehuan == 0) // 仅自动模式下允许设置
		{
			if (stopval == 0 || stopval == 1)
			{
				servo1stop = stopval;
				processed = 1;
				has_response = 1;
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "servo1stop_set", cJSON_CreateNumber(stopval));
				}
			}
		}
		// 手动模式下忽略该命令
	}

	// 检查继电器开关命令
	cJSON *switch_item = cJSON_GetObjectItem(json, "switch");
	if (switch_item != NULL && switch_item->type == cJSON_Number)
	{
		int sw = (int)switch_item->valuedouble;
		if (sw == 0 || sw == 1)
		{
			// 只有状态改变时才更新（避免重复操作）
			if (relay_switch != sw)
			{
				relay_switch = sw;
				Relay_GPIO_Set(relay_switch); // 立即执行GPIO操作
				processed = 1;
				has_response = 1;
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "switch_set", cJSON_CreateNumber(sw));
				}
			}
		}
	}

	// 检查继电器2开关命令
	cJSON *switch1_item = cJSON_GetObjectItem(json, "switch1");
	if (switch1_item != NULL && switch1_item->type == cJSON_Number)
	{
		int sw = (int)switch1_item->valuedouble;
		if (sw == 0 || sw == 1)
		{
			// 只有状态改变时才更新（避免重复操作）
			if (relay_switch1 != sw)
			{
				relay_switch1 = sw;
				Relay2_GPIO_Set(relay_switch1); // 立即执行GPIO操作
				processed = 1;
				has_response = 1;
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "switch1_set", cJSON_CreateNumber(sw));
				}
			}
		}
	}

	// 检查语音控制引脚命令
	cJSON *yuyin_item = cJSON_GetObjectItem(json, "yuyin");
	if (yuyin_item != NULL && yuyin_item->type == cJSON_Number)
	{
		int yy = (int)yuyin_item->valuedouble;
		if (yy == 0 || yy == 1)
		{
			// 只有状态改变时才更新（避免重复操作）
			if (yuyin_state != yy)
			{
				yuyin_state = yy;
				Yuyin_GPIO_Set(yuyin_state); // 立即执行GPIO操作
				processed = 1;
				has_response = 1;
				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "yuyin_set", cJSON_CreateNumber(yy));
					if (yy == 0)
					{
						cJSON_AddItemToObject(unified_response, "description", cJSON_CreateString("Yuyin pin HIGH"));
					}
					else
					{
						cJSON_AddItemToObject(unified_response, "description", cJSON_CreateString("Yuyin pin LOW"));
					}
				}
			}
		}
	}

	// 检查步进电机移动命令（丝杆控制）
	cJSON *motor_item = cJSON_GetObjectItem(json, "motor");
	if (motor_item != NULL && motor_item->type == cJSON_String)
	{
		char *motor_cmd = motor_item->valuestring;
		cJSON *distance_item = cJSON_GetObjectItem(json, "distance");

		if (distance_item != NULL && distance_item->type == cJSON_Number)
		{
			uint32_t distance = (uint32_t)distance_item->valuedouble;
			motor_dir_t dir;

			if (strcmp(motor_cmd, "up") == 0 || strcmp(motor_cmd, "shang") == 0)
			{
				dir = shang;
				Motor_Move(dir, distance);
				processed = 1;
				has_response = 1;

				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "motor_executed", cJSON_CreateString("up"));
					cJSON_AddItemToObject(unified_response, "distance_mm", cJSON_CreateNumber(distance));
				}
			}
			else if (strcmp(motor_cmd, "down") == 0 || strcmp(motor_cmd, "xia") == 0)
			{
				dir = xia;
				Motor_Move(dir, distance);
				processed = 1;
				has_response = 1;

				if (unified_response != NULL)
				{
					cJSON_AddItemToObject(unified_response, "motor_executed", cJSON_CreateString("down"));
					cJSON_AddItemToObject(unified_response, "distance_mm", cJSON_CreateNumber(distance));
				}
			}
		}
	}

	// 检查传统命令格式
	cJSON *command_item = cJSON_GetObjectItem(json, "command");
	if (command_item != NULL && command_item->type == cJSON_String)
	{
		char *command = command_item->valuestring;
		if (Servo_ProcessCommand(command))
		{
			processed = 1;
			has_response = 1;

			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "command_executed", cJSON_CreateString(command));
			}
		}
	}

	// 检查动作命令
	cJSON *action_item = cJSON_GetObjectItem(json, "action");
	if (action_item != NULL && action_item->type == cJSON_String)
	{
		char *action = action_item->valuestring;
		if (strcmp(action, "home") == 0)
		{
			Servo1_Angle = 100.0f;
			Servo2_Angle = 50.0f;
			Servo3_Angle = 90.0f;  // 舵机3默认90度
			Servo4_Angle = 90.0f;
			Servo5_Angle = 0.0f;
			Servo6_Angle = 0.0f;
			processed = 1;
			has_response = 1;

			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "action_executed", cJSON_CreateString("home"));
			}
		}
		else if (strcmp(action, "status") == 0)
		{
			// 返回当前状态（独立响应，不合并）
			cJSON *response = cJSON_CreateObject();
			if (response != NULL)
			{
				cJSON_AddItemToObject(response, "servo1", cJSON_CreateNumber(Servo1_Angle));
				cJSON_AddItemToObject(response, "servo2", cJSON_CreateNumber(Servo2_Angle));
				cJSON_AddItemToObject(response, "servo3", cJSON_CreateNumber(Servo3_Angle));
				cJSON_AddItemToObject(response, "servo4", cJSON_CreateNumber(Servo4_Angle));
				cJSON_AddItemToObject(response, "servo5", cJSON_CreateNumber(Servo5_Angle));
				cJSON_AddItemToObject(response, "servo6", cJSON_CreateNumber(Servo6_Angle));
				cJSON_AddItemToObject(response, "auto", cJSON_CreateNumber(moshiqiehuan));
				cJSON_AddItemToObject(response, "status", cJSON_CreateString("ok"));
			}
			SendJSONResponseSafe(response);
			processed = 1;
			// status命令后不合并，直接发送并跳过unified_response
		}
	}

	// 检查无人机座舱步进电机命令
	cJSON *fly_item = cJSON_GetObjectItem(json, "fly");
	if (fly_item != NULL && fly_item->type == cJSON_Number)
	{
		int fly_val = (int)fly_item->valuedouble;
		if (fly_val == 1)
		{
			/* fly=1：顺时针转1.3圈 */
			Motor_RotateCW_Rounds(1.3f);
			Motor_Stop();
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "fly_set", cJSON_CreateNumber(1));
			}
		}
		else if (fly_val == 0)
		{
			/* fly=0：逆时针转1.5圈回起点 */
			Motor_RotateCCW_Rounds(1.5f);
			Motor_Stop();
			processed = 1;
			has_response = 1;
			if (unified_response != NULL)
			{
				cJSON_AddItemToObject(unified_response, "fly_set", cJSON_CreateNumber(0));
			}
		}
	}

	// 在函数结束前，发送统一响应
	if (has_response && unified_response != NULL)
	{
		SendJSONResponseSafe(unified_response);
	}
	else if (unified_response != NULL)
	{
		cJSON_Delete(unified_response);
	}

	cJSON_Delete(json);
	return processed;
}

/**
  * 函    数：处理串口接收的命令
  * 参    数：无
  * 返 回 值：无
  */
void ProcessSerialCommand(void)
{
	if (Serial_RxFlag == 1)		// 如果有待处理命令
	{
		char local_buffer[100];

		// 从队列中获取命令（支持批量处理）
		while (Serial_GetCommand(local_buffer, sizeof(local_buffer)))
		{
			// 首先判断是否是JSON格式（以'{'开头）
			if (local_buffer[0] == '{')
			{
				// 尝试解析JSON命令（内部已处理错误响应）
				ProcessJSONCommand(local_buffer);
			}
			// 否则尝试处理传统舵机命令
			else if (Servo_ProcessCommand(local_buffer))
			{
				// 传统命令处理成功，发送确认
				cJSON *response = cJSON_CreateObject();
				if (response != NULL)
				{
					cJSON_AddItemToObject(response, "legacy_command_executed", cJSON_CreateString(local_buffer));
					cJSON_AddItemToObject(response, "servo1", cJSON_CreateNumber(Servo1_Angle));
					cJSON_AddItemToObject(response, "servo2", cJSON_CreateNumber(Servo2_Angle));
					cJSON_AddItemToObject(response, "servo3", cJSON_CreateNumber(Servo3_Angle));
					cJSON_AddItemToObject(response, "servo4", cJSON_CreateNumber(Servo4_Angle));
				}
				SendJSONResponseSafe(response);
			}
			else
			{
				// 传统命令也无法识别，发送错误响应
				cJSON *response = cJSON_CreateObject();
				if (response != NULL)
				{
					cJSON_AddItemToObject(response, "error", cJSON_CreateString("Unknown legacy command"));
					cJSON_AddItemToObject(response, "received", cJSON_CreateString(local_buffer));
				}
				SendJSONResponseSafe(response);
			}
		}
	}
}

void SendTempHumidityJSON_Safe(float temperature, float humidity)
{
	Serial_Printf("{\"temp\":%.2f,\"humi\":%.2f}\r\n", temperature, humidity);
}

void SendTempHumidityJSON(float temperature, float humi)
{
	cJSON *json = cJSON_CreateObject();
	cJSON *temp_item = cJSON_CreateNumber(temperature);
	cJSON *humi_item = cJSON_CreateNumber(humi);

	cJSON_AddItemToObject(json, "temp", temp_item);
	cJSON_AddItemToObject(json, "humi", humi_item);

	char *json_string = cJSON_PrintUnformatted(json);
	if (json_string != NULL)
	{
		Serial_Printf("%s\r\n", json_string);
		free(json_string);
	}

	cJSON_Delete(json);
}

void SendServoAnglesJSON(float angle1, float angle2)
{
	SendServoAnglesJSON3(angle1, angle2, Servo_GetAngle3());
}

// 新增：支持三个舵机的JSON回报（只回报角度）
void SendServoAnglesJSON3(float angle1, float angle2, float angle3)
{
	cJSON *json = cJSON_CreateObject();
	cJSON_AddItemToObject(json, "servo1", cJSON_CreateNumber(angle1));
	cJSON_AddItemToObject(json, "servo2", cJSON_CreateNumber(angle2));
	cJSON_AddItemToObject(json, "servo3", cJSON_CreateNumber(angle3));
	char *json_string = cJSON_PrintUnformatted(json);
	if (json_string != NULL)
	{
		Serial_Printf("%s\r\n", json_string);
		free(json_string);
	}
	cJSON_Delete(json);
}

// 新增：简化的舵机角度回报函数（减少内存使用）
void SendServoAnglesJSON4_Safe(float angle1, float angle2, float angle3, float angle4)
{
	// 直接构造JSON字符串，避免动态内存分配
	char json_buffer[128];
	int len = snprintf(json_buffer, sizeof(json_buffer),
		"{\"servo1\":%.1f,\"servo2\":%.1f,\"servo3\":%.1f,\"servo4\":%.1f}\r\n",
		angle1, angle2, angle3, angle4);

	if (len > 0 && len < sizeof(json_buffer))
	{
		Serial_Printf("%s", json_buffer);
	}
	else
	{
		// 如果缓冲区不够，使用更简单的格式
		Serial_Printf("{\"servo_angles\":[%.1f,%.1f,%.1f,%.1f]}\r\n", angle1, angle2, angle3, angle4);
	}
}

// 新增：支持四个舵机的JSON回报（只回报角度）
void SendServoAnglesJSON4(float angle1, float angle2, float angle3, float angle4)
{
	cJSON *json = cJSON_CreateObject();
	cJSON_AddItemToObject(json, "servo1", cJSON_CreateNumber(angle1));
	cJSON_AddItemToObject(json, "servo2", cJSON_CreateNumber(angle2));
	cJSON_AddItemToObject(json, "servo3", cJSON_CreateNumber(angle3));
	cJSON_AddItemToObject(json, "servo4", cJSON_CreateNumber(angle4));
	char *json_string = cJSON_PrintUnformatted(json);
	if (json_string != NULL)
	{
		Serial_Printf("%s\r\n", json_string);
		free(json_string);
	}
	cJSON_Delete(json);
}

// 新增：简化的模式回报函数（减少内存使用）
void SendModeJSON_Safe(void)
{
	Serial_Printf("{\"auto\":%d,\"servo1stop\":%d,\"switch\":%d,\"switch1\":%d,\"yuyin\":%d}\r\n", moshiqiehuan, servo1stop, relay_switch, relay_switch1, yuyin_state);
}

// 新增：舵机5和6独立回报函数（减少串口缓冲压力）
void SendServo56AnglesJSON_Safe(float angle5, float angle6)
{
	Serial_Printf("{\"servo5\":%.1f,\"servo6\":%.1f}\r\n", angle5, angle6);
}

// 新增：单独回报模式
void SendModeJSON(void)
{
	cJSON *mode_json = cJSON_CreateObject();
	cJSON_AddItemToObject(mode_json, "auto", cJSON_CreateNumber(moshiqiehuan));
	cJSON_AddItemToObject(mode_json, "servo1stop", cJSON_CreateNumber(servo1stop));
	cJSON_AddItemToObject(mode_json, "switch", cJSON_CreateNumber(relay_switch));
	cJSON_AddItemToObject(mode_json, "switch1", cJSON_CreateNumber(relay_switch1));
	cJSON_AddItemToObject(mode_json, "yuyin", cJSON_CreateNumber(yuyin_state));
	char *mode_str = cJSON_PrintUnformatted(mode_json);
	if (mode_str != NULL)
	{
		Serial_Printf("%s\r\n", mode_str);
		free(mode_str);
	}
	cJSON_Delete(mode_json);
}

/**
  * 函    数：舵机1自动旋转控制
  * 参    数：无
  * 返 回 值：无
  * 说    明：当moshiqiehuan=0时，控制舵机1在10秒内完成0-180-0度循环
  */
void Servo1_AutoRotate(void)
{
	static float currentAngle = 0.0f;
	static int direction = 0; // 0: 0->180, 1: 180->0
	static uint8_t last_mode = 1;

	// 匀速参数
const float step = 180.0f / 400.0f; // 400周期完成180度(20秒)，速度减半

	if (moshiqiehuan == 0)  // 自动模式
	{
		// 如果刚切换到自动模式，记录当前位置，先归零
		if (last_mode != 0)
		{
			// 进入自动模式，方向先归零
			direction = -1; // -1表示归零阶段
		}

		if (servo1stop == 1)
		{
			// 停止自动旋转，保持当前位置
			// 不改变Servo1_Angle，currentAngle保持
			last_mode = 0;
			return;
		}

		if (direction == -1)
		{
			// 归零阶段，匀速归零
			if (currentAngle > step)
			{
				currentAngle -= step;
				if (currentAngle < 0.0f) currentAngle = 0.0f;
				Servo1_Angle = currentAngle;
			}
			else if (currentAngle < -step)
			{
				currentAngle += step;
				if (currentAngle > 0.0f) currentAngle = 0.0f;
				Servo1_Angle = currentAngle;
			}
			else
			{
				currentAngle = 0.0f;
				Servo1_Angle = 0.0f;
				direction = 0; // 开始0->180
			}
		}
		else if (direction == 0)
		{
			// 0->180
			currentAngle += step;
			if (currentAngle >= 180.0f)
			{
				currentAngle = 180.0f;
				direction = 1;
			}
			Servo1_Angle = currentAngle;
		}
		else if (direction == 1)
		{
			// 180->0
			currentAngle -= step;
			if (currentAngle <= 0.0f)
			{
				currentAngle = 0.0f;
				direction = 0;
			}
			Servo1_Angle = currentAngle;
		}
		last_mode = 0;
	}
	else
	{
		// 切换到手动模式时，记录当前位置
		currentAngle = Servo1_Angle;
		last_mode = 1;
	}
}

void SendSystemStatusJSON_Safe(const char* message)
{
	Serial_Printf("{\"status\":\"%s\"}\r\n", message);
}

void SendSystemStatusJSON(const char* message)
{
	cJSON *json = cJSON_CreateObject();
	cJSON *status_item = cJSON_CreateString(message);

	cJSON_AddItemToObject(json, "status", status_item);

	char *json_string = cJSON_PrintUnformatted(json);
	if (json_string != NULL)
	{
		Serial_Printf("%s\r\n", json_string);
		free(json_string);
	}

	cJSON_Delete(json);
}

// 继电器IO初始化（B8高电平启动）
void Relay_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; // PB8继电器1, PB9继电器2, PB10语音控制
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9); // 继电器默认低电平关闭
	GPIO_SetBits(GPIOB, GPIO_Pin_10); // 语音引脚默认高电平
}

// 继电器控制函数（加延时防干扰）
void Relay_GPIO_Set(uint8_t state)
{
	// 关键段保护：防止继电器切换时的电磁干扰影响中断
	__disable_irq();

	if(state)
		GPIO_SetBits(GPIOB, GPIO_Pin_8); // 高电平启动
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_8); // 低电平关闭

	__enable_irq();

	// 延时等待继电器稳定，避免水泵反向电动势干扰
	Delay_ms(20);
}

// 继电器2控制函数（加延时防干扰）
void Relay2_GPIO_Set(uint8_t state)
{
	// 关键段保护：防止继电器切换时的电磁干扰影响中断
	__disable_irq();

	if(state)
		GPIO_SetBits(GPIOB, GPIO_Pin_9); // 高电平启动
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_9); // 低电平关闭

	__enable_irq();

	// 延时等待继电器稳定，避免水泵反向电动势干扰
	Delay_ms(20);
}

// 语音控制引脚IO初始化（PB10上拉输出）
// 注意：GPIO已在Relay_GPIO_Init中统一初始化，此函数仅作备用
void Yuyin_GPIO_Init(void)
{
	// PB10已在Relay_GPIO_Init中初始化为推挽输出并设置为高电平
	// 此处无需重复初始化，保留此函数以保持接口一致性
}

// 语音控制引脚控制函数
void Yuyin_GPIO_Set(uint8_t state)
{
	if(state == 0)
		GPIO_SetBits(GPIOB, GPIO_Pin_10); // yuyin=0时高电平
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_10); // yuyin=1时低电平
}

// PC13 LED初始化（用于限位触发指示）
void Test_LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13); // 默认关闭（高电平）
}

// PC13 LED控制
void Test_LED_ON(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 低电平点亮
}

void Test_LED_OFF(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13); // 高电平熄灭
}

int main(void)
{
	/*模块初始化*/
	Serial_Init();
	SysTick_Init(72);  // 72MHz，供 tftlcd/touch 用 delay_us/ms
	I2C_GPIO_Config();
	Relay_GPIO_Init(); // 继电器IO初始化
	Yuyin_GPIO_Init(); // 语音控制引脚初始化（PB10）
	StepperMotor_Init(); // 步进电机（丝杆）初始化（限位 PE0/PE1）
	Test_LED_Init(); // PC13 LED初始化（用于限位触发测试）
	Adc_Init();        // ADC液位检测初始化
	Motor_Init();
	Motor_SetSpeed(2);
	Motor_SetStepsPerRev(4096);
	Servo_Init();      // PWM初始化放最后，确保PB5(舵机5)的AF_PP及TIM3重映射不被覆盖

	// 设置初始位置
	Servo_UpdateAngles();	// 更新舵机到初始位置(舵机1,3默认90度，舵机2,4,5,6默认0度)

	// TFTLCD 初始化（背光 PB11；PB0/PB1 留给丝杆步进电机）
	TFTLCD_Init();
	// Display_BootAnimation();  // 开机动画（已取消）
	TP_Init();                // 电阻触摸初始化
	Display_DrawStaticUI();   // 绘制静态看板
	// 开机即绘制按钮（无触摸，pressed=0），避免首次触摸前按钮区空白
	Display_ScanButtons(0, 0, 0, relay_switch, relay_switch1);

	// 发送启动信息
	SendSystemStatusJSON_Safe("Six Servo + Stepper Motor + Liquid Level Control System Started");
	SendSystemStatusJSON_Safe("Temperature & Humidity monitoring enabled (SI7021)");
	SendSystemStatusJSON_Safe("Liquid level detection enabled (PA0 ADC)");
	SendSystemStatusJSON_Safe("Global servo angle variables ready for host control");
	SendSystemStatusJSON_Safe("JSON command support enabled");
	SendSystemStatusJSON_Safe("Supported formats: servo1-6, action:home/status, auto:0/1, motor:up/down");
	SendSystemStatusJSON_Safe("Servo1 auto mode: auto:0 (auto), auto:1 (manual)");
	SendSystemStatusJSON_Safe("Stepper motor: motor:up/down, distance:mm (PUL:PA8, DIR:PB0, ENA:PB1)");
	SendSystemStatusJSON_Safe("Liquid level: PA0 ADC, auto-report every 1s as {yewei:percent}");
	SendSystemStatusJSON_Safe("UAV cabin motor: fly:1 (CW 1.3 rounds), fly:0 (CCW 1.5 rounds) (PB12-15)");

	// 可选：测试全局变量控制（取消注释以启用测试）
	// ExampleDirectServoControl();	// 运行示例控制函数

	// 可选：测试JSON命令解析（取消注释以启用测试）
	// ProcessJSONCommand("{\"servo1\":45}");		// 测试设置舵机1
	// ProcessJSONCommand("{\"action\":\"status\"}");	// 测试状态查询

	// ...existing code...

// 自动回报节奏：1秒内错开发送各类原格式状态，参考铁路版本
static uint32_t tick = 0;
static uint32_t led_blink_counter = 0; // LED闪烁计数器
// 看板缓存（由各 tick 分支更新，每轮主循环刷一次屏；未变化的字段差分刷新会跳过）
static float    disp_temp = 0.0f;
static float    disp_humi = 0.0f;
static uint16_t disp_yewei = 0;

while (1)
{
	// 检测限位触发，LED闪烁1秒（20个周期 x 50ms）
	if (Motor_GetLimitTriggered()) {
		led_blink_counter = 20; // 设置闪烁1秒
	}

	// LED闪烁控制
	if (led_blink_counter > 0) {
		if ((led_blink_counter % 2) == 0) {
			Test_LED_ON();  // 偶数周期点亮
		} else {
			Test_LED_OFF(); // 奇数周期熄灭
		}
		led_blink_counter--;
	} else {
		Test_LED_OFF(); // 闪烁结束，确保LED熄灭
	}

	// 处理串口命令
	ProcessSerialCommand();

	// 舵机1自动旋转控制
	Servo1_AutoRotate();

	// 更新舵机角度
	Servo_UpdateAngles();

	// 自动回报：保持原JSON内容不变，1秒内错开发各类状态包
	if (tick == 0)
	{
		SendServoAnglesJSON4_Safe(Servo_GetAngle1(), Servo_GetAngle2(), Servo_GetAngle3(), Servo_GetAngle4());
	}
	else if (tick == 5)
	{
		SendServo56AnglesJSON_Safe(Servo_GetAngle5(), Servo_GetAngle6());
	}
	else if (tick == 10)
	{
		static uint8_t sensor_fail_count = 0;

		// 温湿度读取加超时保护，避免I2C挂死导致系统卡死
		if (sensor_fail_count < 3)  // 连续失败3次后跳过此传感器
		{
			float temperature = Si7021_Measure(TEMP_NOHOLD_MASTER);
			float humidity = Si7021_Measure(HUMI_NOHOLD_MASTER);

			if (temperature < 100 && humidity < 100 && temperature > -50 && humidity >= 0)
			{
				SendTempHumidityJSON_Safe(temperature, humidity);
				disp_temp = temperature;
				disp_humi = humidity;
				sensor_fail_count = 0;  // 成功后重置计数
			}
			else
			{
				sensor_fail_count++;
				if (sensor_fail_count < 3)
				{
					Serial_Printf("{\"error\":\"SI7021 sensor read error\",\"temp_raw\":%.2f,\"humi_raw\":%.2f}\r\n",
						temperature, humidity);
				}
			}
		}
		// 失败超过3次后静默跳过，不再尝试读取，防止卡死
	}
	else if (tick == 15)
	{
		uint16_t adc_value = Get_Adc_Average(0, 10);  // PA0通道，采样10次求平均
		uint16_t liquid_percent = Get_LiquidLevel_Percent(adc_value);

		// 发送液位检测JSON数据 - 只发送百分比数值
		Serial_Printf("{\"yewei\":%d}\r\n", liquid_percent);
		Serial_Printf("{\"yewei\":%d}\r\n", liquid_percent);
		disp_yewei = liquid_percent;
	}
	else if (tick == 16)
	{
		SendModeJSON_Safe();
	}

	// 继电器状态已在命令处理时立即更新，这里无需重复操作
	// Relay_GPIO_Set(relay_switch);
	// Relay2_GPIO_Set(relay_switch1);

	// 触摸按钮扫描：显示屏优先级最低，与看板刷新错开轮次，避免同轮互相拖累。
	// 按钮点按对采样率要求低，每 100ms（每2轮）扫描一次即可，且刻意避开
	// 数据采集/密集回报的轮次（tick 10/15/16），把总线时间让给串口与传感器。
	if (tick == 3 || tick == 8 || tick == 13 || tick == 18)
	{
		uint8_t pressed = tp_dev.scan(0);
		DisplayButton_t btn = Display_ScanButtons((int16_t)tp_dev.x[0], (int16_t)tp_dev.y[0],
		                                          pressed, relay_switch, relay_switch1);
		if (btn == DISPLAY_BTN_RELAY1)
		{
			relay_switch = relay_switch ? 0 : 1;      // 切换继电器1
			Relay_GPIO_Set(relay_switch);
		}
		else if (btn == DISPLAY_BTN_RELAY2)
		{
			relay_switch1 = relay_switch1 ? 0 : 1;    // 切换继电器2
			Relay2_GPIO_Set(relay_switch1);
		}
	}

	// LCD 看板差分刷新：每秒一次，单独占一轮（tick==4），
	// 避开数据采集(10/15)、串口密集回报(0/5/16)与触摸扫描轮次。
	if (tick == 4)
	{
		Display_UpdateData(Servo_GetAngle1(), Servo_GetAngle2(), Servo_GetAngle3(),
		                   Servo_GetAngle4(), Servo_GetAngle5(), Servo_GetAngle6(),
		                   disp_temp, disp_humi, disp_yewei,
		                   relay_switch, relay_switch1, (moshiqiehuan == 0) ? 1 : 0);
	}

	Delay_ms(50);
	tick++;
	if (tick >= 20) tick = 0; // 1秒循环
}
}

/**
  * 函    数：示例 - 直接通过全局变量控制舵机
  * 参    数：无
  * 返 回 值：无
  * 说    明：展示如何通过全局变量直接控制舵机角度
  */
void ExampleDirectServoControl(void)
{
	// 方法1：直接修改全局变量（推荐用于主机控制）
	Servo1_Angle = 45.0f;		// 设置舵机1到45度
	Servo2_Angle = 135.0f;		// 设置舵机2到135度

	// 方法2：使用新增的设置函数（带范围检查）
	Servo_SetAngle1(60.0f);		// 设置舵机1到60度
	Servo_SetAngle2(120.0f);	// 设置舵机2到120度

	// 注意：修改角度变量后，需要调用Servo_UpdateAngles()来实际驱动舵机
	// 在main函数的主循环中已经定期调用了Servo_UpdateAngles()
// void JSONCommandUsageInfo(void) { /* 仅文档说明，无需实现 */ }
}
