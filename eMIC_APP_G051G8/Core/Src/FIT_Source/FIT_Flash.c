/*
 * FIT_Flash.c
 *
 *  Created on: May 13, 2025
 *      Author: fit0354
 */

#include "FIT_Flash.h"
#include "FIT_DebugMessage.h"
/* Unlock the FLASH control register access */
uint8_t ubFLASH_Unlock(void)
{

	uint8_t sta = 0;

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);
        /* verify Flash is unlock */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
        {
            sta = 1;
        }
    }

    return sta;
}


/* Lock the FLASH control register access */
uint8_t ubFLASH_Lock(void)
{
    uint8_t sta = 1;

    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    /* verify Flash is locked */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00u)
    {
        sta = 0;
    }

    return sta;
}

uint32_t ulGetPage(uint32_t startAddr)
{
    return ((startAddr - FLASH_BASE) / FLASH_PAGE_SIZE);
}

int  FLASH_WaitForLastOperation(uint32_t Timeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */

    uint32_t tickstart = SysTick->VAL;
    uint32_t error = 0;

    while((READ_BIT(FLASH->CR, FLASH_SR_BSY1)))
    {
        if (Timeout != 10000)
        {
            if((Timeout == 0U) || ((SysTick->VAL-tickstart) > Timeout))
            {
                return 1;
            }
        }
    }

    /* check flash errors */
    error = (FLASH->SR & FLASH_SR_ERRORS);

    /* Clear SR register */
    FLASH->SR = FLASH_SR_CLEAR;

    if (error != 0x00U)
    {
        return 2;
    }

    tickstart = SysTick->VAL;
    while ((FLASH->SR & FLASH_SR_CFGBSY) != 0x00U)
    {
    	 if (Timeout != 10000)
		{
			if((Timeout == 0U) || ((SysTick->VAL-tickstart) > Timeout))
			{
				return 3;
			}
		}
    }
    /* There is no error flag set */
    return 0;
}

uint8_t ubFLASH_PageErase(uint32_t page)
{
    uint32_t tmp  = 0;
    uint8_t  result  = 0;

    /* Get configuration register, then clear page number */
    tmp = (FLASH->CR & ~FLASH_CR_PNB);
    FLASH->SR = FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR;
    /* Set page number, Page Erase bit & Start bit */
    FLASH->CR = (tmp | ((page <<  FLASH_CR_PNB_Pos) | FLASH_CR_PER));
    FLASH->CR|=FLASH_CR_STRT;

    result=FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);


    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

    return result;
}

uint8_t ubFLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{

    uint8_t result=0;
	/* Set PG bit */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /* Program first word */
    *(uint32_t *)Address = (uint32_t)Data;

    /* Barrier to ensure programming is performed in 2 steps, in right order
      (independently of compiler optimization behavior) */
    __ISB();

    /* Program second word */
    *(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);

    result=FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    FLASH->CR = 0;


    return result;
}

bool mem_flash_erase(uint32_t addr,bool block)
{
    uint8_t result;
    uint32_t page;

    ubFLASH_Unlock();

    page=ulGetPage(addr);

    result=ubFLASH_PageErase(page);

    if(block)
        ubFLASH_Lock();

    if (result != 0)
    {
	    FT_printf("earse error:%d\r\n",result);
        return false;
    }

    return true;
}

bool mem_flash_write(uint32_t addr, uint8_t *data,uint32_t DataSize)
{
    uint32_t writeAddress=addr;
    uint32_t writeoffset=0;
    uint64_t writeData=0;
    uint8_t result;

    ubFLASH_Unlock();

    while(writeoffset!=DataSize)
    {
        writeData= (uint64_t)data[writeoffset+7]<<56| (uint64_t)data[writeoffset+6]<<48|(uint64_t)data[writeoffset+5]<<40|(uint64_t)data[writeoffset+4]<<32|(uint64_t)data[writeoffset+3]<<24|(uint64_t)data[writeoffset+2]<<16|(uint64_t)data[writeoffset+1]<<8| data[writeoffset];
    	//writeData=0x1122334455667788;
        if(writeAddress%FLASH_PAGE_SIZE==0)
        {
        	mem_flash_erase(writeAddress,false);

            FT_printf("erase:%lx\r\n",writeAddress);

        }

        result=ubFLASH_Program_DoubleWord(writeAddress, writeData);

        if (result==0)
        {
            writeAddress+=8;
            writeoffset+=8;
        }
        else
        {
    	    FT_printf("flash error:%d\r\n",result);
            ubFLASH_Lock();
            return false;
        }
    }

    ubFLASH_Lock();

    return true;
}



