/*
 * FIT_LED.h
 *
 *  Created on: Jun 24, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_LED_H_
#define SRC_FIT_HEADER_FIT_LED_H_

#define LED_TUBE_Pin LL_GPIO_PIN_1
#define LED_TUBE_GPIO_Port GPIOB

#define LED_HUB_Pin LL_GPIO_PIN_8
#define LED_HUB_GPIO_Port GPIOA

#define LED_NOTIFY_TYPE_NONE                                             0x00
#define LED_NOTIFY_TYPE_PCMD_INIT_FAIL                                   0x01
#define LED_NOTIFY_TYPE_PEOGRAMMING                                      0x02
#define LED_NOTIFY_TYPE_I2C_TIMEOUT                                      0x03
#define LED_NOTIFY_TYPE_ASI_BUS_CLOCK_ERROR                              0x04
#define LED_NOTIFY_TYPE_PLL_CLOCK_ERROR                                  0x05
#define LED_NOTIFY_TYPE_ASI_INPUT_MIXING_SATURATION_ERROR                0x06
#define LED_NOTIFY_TYPE_VAD_POWER_UP_DETECT                              0x07
#define LED_NOTIFY_TYPE_VAD_POWER_DOWN_DETECT                            0x08

#define LED_ACTION_TYPE_NONE                   0x00
#define LED_ACTION_TYPE_BLINKING               0x01
#define LED_ACTION_TYPE_BLINKING_PROGRAMMING   0x02
#define LED_ACTION_TYPE_BLINKING_ERROR         0x03

void initLED();
void ResetLED_Group();
void TurnOnTubeLED();
void TurnOnHubLED();
void setLedNotify(uint8_t event);
uint8_t getLedNotifyType();
void setLEDBlinkingProgramming(uint8_t actionType,uint32_t interval);
void setLEDBlinkingSlowly(uint8_t actionType,uint32_t interval,uint32_t interval2,int times);
void resetLEDParamater();
void processLED();



#endif /* SRC_FIT_HEADER_FIT_LED_H_ */
