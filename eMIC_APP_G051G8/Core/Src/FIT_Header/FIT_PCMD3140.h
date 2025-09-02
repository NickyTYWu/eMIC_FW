/*
 * FIT_PCMD3140.h
 *
 *  Created on: May 19, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_PCMD3140_H_
#define SRC_FIT_HEADER_FIT_PCMD3140_H_

#define PCMD3140_INT_Pin LL_GPIO_PIN_4
#define PCMD3140_INT_GPIO_Port GPIOB

#define PCMD3140_SLAVE_ADDR 0x9c

#define PAGE0_REG_MAX_SIZE 48
#define PAGE1_REG_MAX_SIZE 2
#define PAGE2_REG_MAX_SIZE 121
#define PAGE3_REG_MAX_SIZE 121
#define PAGE4_REG_MAX_SIZE 77

struct PMD3140_Register{
    uint8_t regAddress;
    uint8_t data;
};

void init_PCMD3140();
void setPage(uint8_t page);
void dumpPage(uint8_t page);
bool isUnUsedRegister(uint8_t reg);
int8_t getPageFromSRAM(uint8_t page,uint8_t *pageBuf,uint8_t pageSize);
uint8_t writePCMD3140Page(uint8_t page,uint8_t *pageBuf,uint8_t bufSize,bool bypassChecksum);
bool resetToDefault(uint8_t page);
bool writeRegToPCDM3140(uint8_t *cmd);
bool writeRegToPCDM3140WithPageParameters(uint8_t *cmd);
uint8_t readRegFromPCDM3140(uint8_t *cmd);
uint8_t readRegFromPCDM3140WithPageParameters(uint8_t *cmd);
bool isPCMD3140DataCorrect();
void init_PCMD3140_Interrupt();
void PCMD3140_Interrupt_cb();
void processPCMD3140();
#endif /* SRC_FIT_HEADER_FIT_PCMD3140_H_ */
