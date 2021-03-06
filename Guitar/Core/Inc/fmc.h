#ifndef FMC_H
#define FMC_H

#include "stm32f7xx_hal.h"

#define MEM_SWAPPING 0
#if MEM_SWAPPING
#define NORSRAM_BASE_ADDRESS 0xC0000000
#else
#define NORSRAM_BASE_ADDRESS 0x60000000
#endif

#define MIN_ADDSET_TIME  0
#define MAX_ADDSET_TIME  15
#define MIN_ADDHOLD_TIME 1
#define MAX_ADDHOLD_TIME 15
#define MIN_DATAST_TIME  1
#define MAX_DATAST_TIME  255

#define NORSRAM_BANK_BASE_ADDRESS(x) NORSRAM_BASE_ADDRESS + ((x-1)*0x10000000)

/* LCD */
#define FMC_LCD_8BIT_SEND_COMMAND(bank, command) *(__IO uint8_t *)(NORSRAM_BANK_BASE_ADDRESS(bank)) = command;
#define FMC_LCD_16BIT_SEND_COMMAND(bank, command) *(__IO uint16_t *)(NORSRAM_BANK_BASE_ADDRESS(bank)) = command;
#define FMC_LCD_8BIT_SEND_DATA(bank, a_x, data)  *(__IO uint8_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (1 << a_x)) = data;
#define FMC_LCD_16BIT_SEND_DATA(bank, a_x, data) *(__IO uint16_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (1 << (a_x + 1))) = data;

/* SRAM
 *
 * from pg 337 of STM32F76xx datasheet
 *
 * Memory width | Data address issued to memory
 * ---------------------------------------------
 * 8bit         | HADDR[25:0]
 * 16bit        | HADDR[25:1] >> 1
 * 32bit        | HADDR[25:2] >> 2
 *
 * */
#define FMC_SRAM_8BIT_SEND_DATA(bank, address, data)  *(__IO uint8_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + address) = data;
#define FMC_SRAM_8BIT_READ_DATA(bank, address) *(__IO uint8_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + address);
#define FMC_SRAM_16BIT_SEND_DATA(bank, address, data) *(__IO uint16_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (address<<1)) = data;
#define FMC_SRAM_16BIT_READ_DATA(bank, address) *(__IO uint16_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (address<<1));
#define FMC_SRAM_32BIT_SEND_DATA(bank, address, data) *(__IO uint32_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (address<<2)) = data;
#define FMC_SRAM_32BIT_READ_DATA(bank, address) *(__IO uint32_t *)(NORSRAM_BANK_BASE_ADDRESS(bank) + (address<<2));

#endif
