/*
 * FIT_DebugMessage.h
 *
 *  Created on: May 12, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_DEBUGMESSAGE_H_
#define SRC_FIT_HEADER_FIT_DEBUGMESSAGE_H_

#define Debug_TX_Pin LL_GPIO_PIN_2
#define Debug_TX_GPIO_Port GPIOA
#define Debug_RX_Pin LL_GPIO_PIN_3
#define Debug_RX_GPIO_Port GPIOA

#define ACK_CMD             0x00

#define CMD_FAIL            0x00
#define CMD_SUCCESS         0x01

#define RCV_CMD_MAX_LEN 132

void proccess_rxbuf();
uint8_t PreparePacketToParse();
uint8_t ProcessPacket();
uint8_t Read_Data();
int DataIsValid();
void init_DebugMessage();
void processDebugRX();
void dumpRXbuf();
void FT_printf(const char *fmt, ...);
void FTM_printf(const char *fmt, ...);
void SendData(uint8_t *data,uint8_t dataLen);
void GenCommand(uint8_t nCMD, uint8_t *pCMD, uint16_t nCmdLen);
void ResponseACK(uint8_t CMD,uint8_t bSuccess);
void enterUpgrade();
void getMode();
void responeResetSuccess();
int8_t processWritePage(uint8_t *buf);
void responsePagefromPCMD3140(uint8_t page);
void responsePagefromEEprom(uint8_t blockID);
void responseEEpromOneByteRead(uint16_t addr);
void responeReadModelNumber(uint8_t *cmd);
void ResponseDeviceDebugMessage(uint8_t *buf,uint8_t len);
bool enableFWLog(uint8_t bEnable);
bool enableWatchdog(uint8_t bEnable);
uint8_t calculateChecksum(uint8_t *data,uint8_t len);
bool isbEnterUpgradeMode();
void responeSensirionRaw(uint16_t temperatureRaw,uint16_t humidityRaw,uint8_t result);
void notifySensirion(int32_t temperature, int32_t humidity);
void responseReadDPS368TempPressure(uint8_t *tempResult,uint8_t *pressureResult,uint8_t tempOSR,uint8_t pressureOSR);
//void mini_printf(const char *fmt, ...);
//void itoa_simple_hex(uint32_t value, char *buf);
//void itoa_simple_signed(int value, char *buf);
#endif /* SRC_FIT_HEADER_FIT_DEBUGMESSAGE_H_ */
