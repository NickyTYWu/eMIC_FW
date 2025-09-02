/*
 * FIT_sensirion_hw_i2c_implementation.h
 *
 *  Created on: May 12, 2025
 *      Author: fit0354
 */

#ifndef SRC_FIT_HEADER_FIT_SENSIRION_HW_I2C_IMPLEMENTATION_H_
#define SRC_FIT_HEADER_FIT_SENSIRION_HW_I2C_IMPLEMENTATION_H_

#define I2C2_SCL_Pin LL_GPIO_PIN_11
#define I2C2_SCL_GPIO_Port GPIOA
#define I2C2_SDA_Pin LL_GPIO_PIN_12
#define I2C2_SDA_GPIO_Port GPIOA

void sensirion_i2c_init(void);
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count);
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,uint16_t count);
void sensirion_sleep_usec(uint32_t useconds);
int8_t FIT_I2C2_Master_Receive(uint8_t device_id , uint8_t *pdata, uint8_t size,uint16_t timeout);
int8_t FIT_I2C2_Master_Transmit(uint8_t device_id ,uint8_t *pdata, uint8_t size,uint16_t timeout);
void getTemperatureAndHumidity();
void getTemperatureAndHumidity(int32_t *temperature,int32_t *humidity);
#endif /* SRC_FIT_HEADER_FIT_SENSIRION_HW_I2C_IMPLEMENTATION_H_ */
