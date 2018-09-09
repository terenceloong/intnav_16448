#include "stm32f4xx.h"

void flash_read(uint32_t *dst, uint32_t size)
{
    uint32_t address = 0x080E0000; //Sector 11, 128KB
    uint32_t i;

    for(i=0; i<size; i++)
    {
        *(dst++) = *(__IO uint32_t*)address;
        address += 4;
    }
}

void flash_write(uint32_t *src, uint32_t size)
{
    uint32_t address = 0x080E0000;
    uint32_t i;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
    if(FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
        while(1);
    for(i=0; i<size; i++)
    {
        if(FLASH_ProgramWord(address, *(src++)) == FLASH_COMPLETE)
            address += 4;
    }
    FLASH_Lock();
}
