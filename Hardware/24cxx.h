#ifndef __24CXX_H
#define __24CXX_H

#include "system.h"

/*
 * 说明：本工程没有外置 AT24CXX EEPROM 芯片。
 *      为了让 touch.c 的 API 调用保持不变，这里提供一组同名 API，
 *      内部用 STM32F103 的内部 Flash 最后一页（0x0807F800, 大小 2KB）
 *      模拟一个 256 字节的 EEPROM。
 *
 * 触摸校准实际只用到 0~13 字节（详见 touch.c 中的 SAVE_ADDR_BASE = 200 起 14 字节）。
 *
 * 注意：
 *   - 写操作会执行 一次擦除 + 多次半字编程，耗时数十毫秒，仅在校准完成时调用，影响很小。
 *   - 内部 Flash 擦写次数 ≥ 10000 次，校准这种偶尔操作完全够用。
 */

/* 模拟 EEPROM 的容量（字节）。256 已远超 touch.c 的实际需求（14 字节）。 */
#define AT24CXX_FAKE_SIZE   256

/* AT24CXX 标准 API（保持与原驱动同名同签名，touch.c 一行不用改） */
void  AT24CXX_Init(void);
u8    AT24CXX_ReadOneByte(u16 ReadAddr);
void  AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite);
void  AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len);
u32   AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len);
void  AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite);
void  AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead);
u8    AT24CXX_Check(void);

#endif
