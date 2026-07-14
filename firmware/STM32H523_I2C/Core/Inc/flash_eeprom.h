#ifndef _EMULATION_EE_H
#define _EMULATION_EE_H
#include "stm32h5xx.h"//
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#define STM32H523_FLASH_SIZE 512

#define MCU_PAGE_SIZE	(uint16_t)0x2000	// high density one page =8Kbyte=2048X4

#define Bootloader_START_ADDRESS	((uint32_t)0x08000000)// only inside page0 size 24k page0---page2
#define CODE_A_START_ADDRESS	((uint32_t)0x08006000)//capacity is page3---page32 total 240K
#define CODE_B_START_ADDRESS	((uint32_t)0x08042000)//capacity is page33---page62 total 240K
#define SYS_START_ADDRESS	((uint32_t)0x0807E000)//Last page address of page63 sys 4k config addr
#define CAL_START_ADDRESS	((uint32_t)0x0807F000)//4k calibration parameter address


uint32_t Read_Word(uint32_t Addr);
void FLASH_ReadData(uint32_t Readaddr, uint32_t *pBuffer, uint16_t Num);
void FLASH_WriteData(uint32_t Readaddr, uint32_t *pBuffer, uint16_t Num);//Num must be 4 X i
void FLASH_Program_QUADWORD(uint32_t WriteAddr, uint32_t pBuffer0);
void FLASH_Erase_Page(uint8_t N, uint8_t N_page, uint8_t N_bank);
#endif
//Bank1 page0---31 and Bank2 page0---31
//flash calibration rules 0x5A is start byte and 0xA5 is end byte
//CRC do include start byte
//LPUART1 receive buffer is calibration_uart_rx_buffer[155];//calibration buffer length is 155 and EX:5A XX XX CRC A5
//flash calibration rules 0x5C is start byte and 0xC5 is end byte
//CRC do include start byte
//LPUART1 receive buffer is B_uart_rx_buffer[2048 + 4];//update code buffer length is 2048 and EX:5C XX XX XX XX XX XX XX XX YY YY YY YY YY YY YY YY index total_index CRC C5
//update code has index from 0 to n
