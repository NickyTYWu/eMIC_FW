/*
 * FIT_eeprom_pcmd3140_hw_i2c_implemention.h
 *
 *  Created on: May 16, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_EEPROM_PCMD3140_HW_I2C_IMPLEMENTION_H_
#define SRC_FIT_HEADER_FIT_EEPROM_PCMD3140_HW_I2C_IMPLEMENTION_H_

#define I2C1_SCL_Pin LL_GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin LL_GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

void eeprom_pcmd3140_i2c_init(void);
int8_t eeprom_pcmd3140_i2c_read(uint8_t address, uint8_t* data, uint16_t count);
int8_t eeprom_pcmd3140_i2c_write(uint8_t address, const uint8_t* data,uint16_t count);
void eeprom_pcmd3140_sleep_usec(uint32_t useconds);
int8_t FIT_I2C1_Master_Transmit(uint8_t device_id ,uint8_t *pdata, uint8_t size,uint16_t timeout);
int8_t FIT_I2C1_Master_Receive(uint8_t device_id , uint8_t *pdata, uint8_t size,uint16_t timeout);
#endif /* SRC_FIT_HEADER_FIT_EEPROM_PCMD3140_HW_I2C_IMPLEMENTION_H_ */
