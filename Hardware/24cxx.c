#include "24cxx.h"
#include "stm32f10x_flash.h"
#include <string.h>

/*
 * AT24CXX 兼容层 —— 内部 Flash 实现
 *
 * STM32F103ZE (高密度) Flash 布局：
 *   - 总容量 512KB，起始 0x08000000，结束 0x0807FFFF
 *   - 页大小 2KB（高密度）
 *   - 我们独占最后一页：0x0807F800 ~ 0x0807FFFF
 *
 * 数据组织：
 *   - 用一块 RAM 缓冲区 g_eep_cache[AT24CXX_FAKE_SIZE]，对外读写均针对它
 *   - 启动时从 Flash 拷贝到缓冲区（首次使用时全 0xFF）
 *   - 写入时：修改缓冲区 -> 擦除最后一页 -> 把缓冲区写回 Flash
 *
 * 校验机制：
 *   - 校准数据由 touch.c 自带 magic 0x0A 标志，无需在本层做额外校验
 */

/* Flash 最后一页起始地址（STM32F103xE：256KB 以上型号页大小 2KB） */
#define EEP_FLASH_PAGE_ADDR   ((uint32_t)0x0807F800)
#define EEP_FLASH_PAGE_SIZE   2048u

/* RAM 缓冲：对应"虚拟 EEPROM"内容 */
static u8  g_eep_cache[AT24CXX_FAKE_SIZE];
static u8  g_eep_inited = 0;

/* 把 Flash 最后一页前 AT24CXX_FAKE_SIZE 字节读到 RAM 缓冲 */
static void eep_load_from_flash(void)
{
	u16 i;
	const volatile u8 *p = (const volatile u8 *)EEP_FLASH_PAGE_ADDR;
	for (i = 0; i < AT24CXX_FAKE_SIZE; i++)
	{
		g_eep_cache[i] = p[i];
	}
}

/* 把 RAM 缓冲整页写回 Flash（先擦除整页，再按半字写入） */
static void eep_flush_to_flash(void)
{
	u16 i;
	uint16_t halfword;
	FLASH_Status status;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	/* 1. 擦除最后一页 */
	status = FLASH_ErasePage(EEP_FLASH_PAGE_ADDR);
	if (status != FLASH_COMPLETE)
	{
		FLASH_Lock();
		return;
	}

	/* 2. 把缓冲区按半字写入（STM32F1 Flash 编程粒度为 16-bit） */
	for (i = 0; i < AT24CXX_FAKE_SIZE; i += 2)
	{
		halfword = (uint16_t)g_eep_cache[i] | ((uint16_t)g_eep_cache[i + 1] << 8);
		status = FLASH_ProgramHalfWord(EEP_FLASH_PAGE_ADDR + i, halfword);
		if (status != FLASH_COMPLETE)
		{
			break;
		}
	}

	FLASH_Lock();
}

/* 公共 API ----------------------------------------------------------------- */

void AT24CXX_Init(void)
{
	if (!g_eep_inited)
	{
		eep_load_from_flash();
		g_eep_inited = 1;
	}
}

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	if (!g_eep_inited) AT24CXX_Init();
	if (ReadAddr >= AT24CXX_FAKE_SIZE) return 0xFF;
	return g_eep_cache[ReadAddr];
}

void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	if (!g_eep_inited) AT24CXX_Init();
	if (WriteAddr >= AT24CXX_FAKE_SIZE) return;
	if (g_eep_cache[WriteAddr] == DataToWrite) return; /* 无变化，省一次擦写 */
	g_eep_cache[WriteAddr] = DataToWrite;
	eep_flush_to_flash();
}

/*
 * 把 DataToWrite 拆成 Len 字节（小端），写入 WriteAddr 开始的位置
 * 与原 AT24CXX 驱动行为完全一致
 */
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len)
{
	u8 t;
	u8 dirty = 0;

	if (!g_eep_inited) AT24CXX_Init();
	if (WriteAddr + Len > AT24CXX_FAKE_SIZE) return;

	for (t = 0; t < Len; t++)
	{
		u8 b = (u8)((DataToWrite >> (8 * t)) & 0xFF);
		if (g_eep_cache[WriteAddr + t] != b)
		{
			g_eep_cache[WriteAddr + t] = b;
			dirty = 1;
		}
	}
	if (dirty) eep_flush_to_flash();
}

u32 AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len)
{
	u8 t;
	u32 temp = 0;

	if (!g_eep_inited) AT24CXX_Init();
	if (ReadAddr + Len > AT24CXX_FAKE_SIZE) return 0;

	for (t = 0; t < Len; t++)
	{
		temp <<= 8;
		temp += g_eep_cache[ReadAddr + Len - 1 - t];
	}
	return temp;
}

void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
{
	u16 i;
	u8 dirty = 0;

	if (!g_eep_inited) AT24CXX_Init();
	if (WriteAddr + NumToWrite > AT24CXX_FAKE_SIZE) return;

	for (i = 0; i < NumToWrite; i++)
	{
		if (g_eep_cache[WriteAddr + i] != pBuffer[i])
		{
			g_eep_cache[WriteAddr + i] = pBuffer[i];
			dirty = 1;
		}
	}
	if (dirty) eep_flush_to_flash();
}

void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{
	u16 i;
	if (!g_eep_inited) AT24CXX_Init();
	if (ReadAddr + NumToRead > AT24CXX_FAKE_SIZE) return;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = g_eep_cache[ReadAddr + i];
	}
}

/* 兼容原驱动的"自检"接口：内部 Flash 模拟，永远返回 0（OK） */
u8 AT24CXX_Check(void)
{
	if (!g_eep_inited) AT24CXX_Init();
	return 0;
}
