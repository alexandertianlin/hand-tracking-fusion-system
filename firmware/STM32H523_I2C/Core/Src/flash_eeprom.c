#include "flash_eeprom.h"

uint32_t SectorError = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;

//read flash data start
uint32_t Read_Word(uint32_t Addr)
{
uint32_t data;
data = *(__IO uint32_t *)Addr;//addr to point
return data;
}
void FLASH_ReadData(uint32_t ReadAddr, uint32_t *pBuffer, uint16_t Num)
{

	uint16_t i;
	for(i=0;i<Num;i++)
	{
		pBuffer[i] = Read_Word(ReadAddr);
		ReadAddr+=4;
	}

}
void FLASH_WriteData(uint32_t WriteAddr, uint32_t *pBuffer, uint16_t Num)//Num must be 4 X i
{

	uint16_t i;
	for(i=0;i<Num/4;i++)
	{
		FLASH_Program_QUADWORD(WriteAddr, (uint32_t)&pBuffer[4*i]);
		WriteAddr+=16;
	}

}
void FLASH_Program_QUADWORD(uint32_t WriteAddr, uint32_t pBuffer0)
{
	/*
	   uint32_t pBuffer0[4] = {0x12345678,
	    0x87654321,
	    0x12344321,
	    0x56788765}
	*/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, WriteAddr, ((uint32_t)pBuffer0)) == HAL_OK)
		    {
			__NOP();
		    }

}
void FLASH_Erase_Page(uint8_t N, uint8_t N_page, uint8_t N_bank)
{

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.Banks         = N_bank;//bank 1(page0---31) or 2(page0---31)
	  EraseInitStruct.Sector        = N_page;//start from which page to erase
	  EraseInitStruct.NbSectors     = N;//erase total number

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
		{
			__NOP();
		}



}
