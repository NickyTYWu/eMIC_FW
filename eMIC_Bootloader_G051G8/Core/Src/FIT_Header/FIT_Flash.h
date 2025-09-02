/*
 * FIT_Flash.h
 *
 *  Created on: May 13, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_FLASH_H_
#define SRC_FIT_HEADER_FIT_FLASH_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define FLASH_OPT_OVERTIMER             (0xfFFFF)
#define FLASH_OPT_TRY_COUNT             (5)

#define APPLICATION_ADDRESS     (uint32_t)0x08002000
#define INFO_ADDRESS            (uint32_t)0x0800F800
#define FLASH_PAGE_SIZE                 0x00000800U    /*!< FLASH Page Size, 2 KBytes */
#define INAPP        0x01
#define INBOOTLOADER 0x02

#define FLASH_KEY1                      0x45670123U   /*!< Flash key1 */
#define FLASH_KEY2                      0xCDEF89ABU   /*!< Flash key2: used with FLASH_KEY1
                                                           to unlock the FLASH registers access */
#if defined(FLASH_PCROP_SUPPORT)
#define FLASH_SR_ERRORS                 (FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
                                         FLASH_SR_PGAERR | FLASH_SR_SIZERR  | FLASH_SR_PGSERR |  \
                                         FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR |   \
                                         FLASH_SR_OPTVERR) /*!< All SR error flags */
#else
#define FLASH_SR_ERRORS                 (FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
                                         FLASH_SR_PGAERR | FLASH_SR_SIZERR  | FLASH_SR_PGSERR |  \
                                         FLASH_SR_MISERR | FLASH_SR_FASTERR |                    \
                                         FLASH_SR_OPTVERR) /*!< All SR error flags */
#endif /* FLASH_PCROP_SUPPORT */
#define FLASH_SR_CLEAR                  (FLASH_SR_ERRORS | FLASH_SR_EOP)

#define HAL_MAX_DELAY               0xFFFFFFFFU
#define FLASH_TIMEOUT_VALUE         50000U /* 50 s */

uint8_t ubFLASH_Unlock(void);
uint8_t ubFLASH_Lock(void);
uint32_t ulGetPage(uint32_t startAddr);
uint8_t ubFLASH_PageErase(uint32_t page);
int  FLASH_WaitForLastOperation(uint32_t Timeout);
uint8_t ubFLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);
bool mem_flash_erase(uint32_t addr,bool block);
bool mem_flash_write(uint32_t addr, uint8_t *data,uint32_t DataSize);
#endif /* SRC_FIT_HEADER_FIT_FLASH_H_ */
