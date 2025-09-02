/*
 * FIT_eeprom_pcmd3140_hw_i2c_implemention.c
 *
 *  Created on: May 16, 2025
 *      Author: fit0354
 */
#include "main.h"
#include "FIT_eeprom_pcmd3140_hw_i2c_implemention.h"
#include "FIT_DebugMessage.h"
#include "FIT_LED.h"
/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void eeprom_pcmd3140_i2c_init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    LL_I2C_InitTypeDef I2C_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    /**I2C1 GPIO Configuration
	  PB6   ------> I2C1_SCL
	  PB7   ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = I2C1_SCL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */

    /** I2C Initialization*/

    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.Timing = 0x00503D58;
    I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    I2C_InitStruct.DigitalFilter = 0;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_EnableAutoEndMode(I2C1);
    LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t eeprom_pcmd3140_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    return (int8_t)FIT_I2C1_Master_Receive((uint16_t)(address),data, count, 100);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t eeprom_pcmd3140_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
    return (int8_t)FIT_I2C1_Master_Transmit((uint16_t)(address),(uint8_t*)data, count, 300);
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void eeprom_pcmd3140_sleep_usec(uint32_t useconds)
{
    uint32_t msec = useconds / 1000;
    if (useconds % 1000 > 0) {
        msec++;
    }

    LL_mDelay(msec);
}

int8_t FIT_I2C1_Master_Transmit(uint8_t device_id ,uint8_t *pdata, uint8_t size,uint16_t timeout)
{
    // timeout counter
    uint16_t cnt = 0;

    while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET)
    {
        LL_mDelay(1);
        if(cnt++ > timeout)
        {
            FT_printf("i2c1 timeout\r\n");
         	setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
            eeprom_pcmd3140_i2c_init();
            return -1;
        }
    }

    LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    for(uint8_t i = 0; i < size; i++)
    {
        while(LL_I2C_IsActiveFlag_TXE(I2C1)== RESET)
        {
            LL_mDelay(1);
            if(cnt++ > timeout)
            {
    	        FT_printf("i2c1 timeout\r\n");
    	     	setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
    	        eeprom_pcmd3140_i2c_init();
                return -1;
            }
        }
        LL_I2C_TransmitData8(I2C1, *pdata++);
    }

    while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET)
    {
        LL_mDelay(1);
        if(cnt++ > timeout)
        {
    	    FT_printf("i2c1 timeout\r\n");
    	 	setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
    	    eeprom_pcmd3140_i2c_init();
            return -1;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    (I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));

    return 0;
}

int8_t FIT_I2C1_Master_Receive(uint8_t device_id , uint8_t *pdata, uint8_t size,uint16_t timeout)
{
    // timeout counter
    uint16_t cnt = 0;

    while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET)
    {
        LL_mDelay(1);
        if(cnt++ > timeout)
        {
          FT_printf("i2c1 timeout\r\n");
       	  setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
          eeprom_pcmd3140_i2c_init();
          return -1;
        }
    }

    LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    for(uint8_t i = 0; i < size; i++)
    {
        while(LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET)
        {
            LL_mDelay(1);
            if(cnt++ > timeout)
            {
                FT_printf("i2c1 timeout\r\n");
             	setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
                eeprom_pcmd3140_i2c_init();
                return -1;
            }
        }

        *pdata++ = LL_I2C_ReceiveData8(I2C1);
    }

    while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET)
    {
        LL_mDelay(1);
        if(cnt++ > timeout)
        {
            FT_printf("i2c1 timeout\r\n");
         	setLedNotify(LED_NOTIFY_TYPE_I2C_TIMEOUT);
            eeprom_pcmd3140_i2c_init();
            return -1;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    (I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));

    return 0;
}



