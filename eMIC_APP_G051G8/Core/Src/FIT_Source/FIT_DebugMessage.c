/*
 * FIT_DebugMessage.c
 *
 *  Created on: May 12, 2025
 *      Author: fit0354
 */
#include "main.h"
#include "FIT_DebugMessage.h"
#include "FIT_FLASH.h"
#include "FIT_Upgrade.h"
#include "FIT_PCMD3140.h"
#include "FIT_M24C08_EE.h"
#include "FIT_sensirion_hw_i2c_implementation.h"
#include "FIT_eeprom_pcmd3140_hw_i2c_implemention.h"
#include "FIT_LED.h"
#include "Command.h"
#include "ftm.h"
#include "ftm_ringbuff.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

uint8_t RXBuffer[512];
uint8_t RCV_PACKET[192];
uint8_t rxbuf_index=0;
uint8_t temprxbuf_index=0;
uint8_t rcv_index=0;
extern int version;
bool bEnterUpgradeMode=false;
bool bShowDebugMSG=false;
extern RingBuffer rb;

void init_DebugMessage()
{
    /* USER CODE BEGIN USART2_Init 0 */
	rxbuf_index=0;
	temprxbuf_index=0;
	rcv_index=0;
    /* USER CODE END USART2_Init 0 */

    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /**USART2 GPIO Configuration
      PA2   ------> USART2_TX
      PA3   ------> USART2_RX
    */
    GPIO_InitStruct.Pin = Debug_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(Debug_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Debug_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(Debug_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);

    /* USER CODE BEGIN WKUPType USART2 */

    /* USER CODE END WKUPType USART2 */

    LL_USART_Enable(USART2);

    LL_USART_EnableIT_RXNE(USART2);
    //LL_USART_EnableIT_ERROR(USART2);

    if(readEnableDebugMsgFlag()==0x01)
    {
    	bShowDebugMSG=true;
    }
    else
	{
		bShowDebugMSG=false;
	}
    //func_init();
#if 0
    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /**LPUART1 GPIO Configuration
     PA2   ------> LPUART1_TX
     PA3   ------> LPUART1_RX
    */
    GPIO_InitStruct.Pin = Debug_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(Debug_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Debug_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(Debug_RX_GPIO_Port, &GPIO_InitStruct);

    NVIC_SetPriority(USART3_4_LPUART1_IRQn, 0);
    NVIC_EnableIRQ(USART3_4_LPUART1_IRQn);

    LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV1;
    LPUART_InitStruct.BaudRate = 115200;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_DisableFIFO(LPUART1);

    LL_LPUART_Enable(LPUART1);

    LL_LPUART_EnableIT_RXNE(USART2);
    LL_LPUART_EnableIT_ERROR(USART2);
#endif
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    /* Wait for TXE flag to be raised */
    //while (!LL_LPUART_IsActiveFlag_TXE(LPUART1)){}
	//LL_LPUART_TransmitData8(LPUART1,ch);
	//while(!LL_LPUART_IsActiveFlag_TC(LPUART1)){}

	 while (!LL_USART_IsActiveFlag_TXE(USART2)){}
	 LL_USART_TransmitData8(USART2,ch);
	 while(!LL_USART_IsActiveFlag_TC(USART2)){}

    return ch;
}

bool isbEnterUpgradeMode()
{
	return bEnterUpgradeMode;
}

void dumpRXbuf()
{
    for(int i=0;i<rxbuf_index;i++)
    {
    	FT_printf("%c",RXBuffer[i]);
    }
    FT_printf("\r\n");
}


int DataIsValid()
{
    return(( rxbuf_index != temprxbuf_index)? 1:0 );
}

void put_s(char *buf,int len)
{
	//("%d %x\r\n",len,buf[0]);
	for(int i=0;i<len;i++)
	{
	    //while (!LL_LPUART_IsActiveFlag_TXE(LPUART1)){}
	    //LL_LPUART_TransmitData8(LPUART1,buf[i]);
	    //while(!LL_LPUART_IsActiveFlag_TC(LPUART1)){}
	    while (!LL_USART_IsActiveFlag_TXE(USART2)){}
	    LL_USART_TransmitData8(USART2,buf[i]);
	    while(!LL_USART_IsActiveFlag_TC(USART2)){}
	}
}


void FTM_printf(const char *fmt, ...)
{
	va_list args;
    char buffer[128]={0};
    va_start(args,fmt);
    vsprintf(buffer,fmt,args);
    va_end(args);
    put_s(buffer,strlen(buffer));
}

void FT_printf(const char *fmt, ...)
{
    if(!bShowDebugMSG)
    {
    	return;
    }

	va_list args;
    char buffer[128]={0};
    va_start(args,fmt);
    vsprintf(buffer,fmt,args);
    va_end(args);
    //put_s(buffer,strlen(buffer));
    ResponseDeviceDebugMessage((uint8_t*) buffer,strlen(buffer));
}

uint8_t Read_Data()
{
    if(DataIsValid())
    {
        uint8_t data;
        data=RXBuffer[temprxbuf_index];
        temprxbuf_index++;
        if(temprxbuf_index==512)
            temprxbuf_index=0;
        return data;
    }
    return 0;
}


void SendData(uint8_t *data,uint8_t dataLen)
{
	put_s((char*)data,dataLen);
}

uint8_t calculateChecksum(uint8_t *data,uint8_t len)
{
    uint8_t sum=0;

    for(int i=0;i<len;i++)
        sum+=data[i];

    return (uint8_t)(0x100-sum);
}

void GenCommand(uint8_t nCMD, uint8_t *pCMD, uint16_t nCmdLen)
{
    int n_Len = 0;
    uint8_t m_Linkbuffer_Out[256];

    m_Linkbuffer_Out[0] = (uint8_t)0x55;
    m_Linkbuffer_Out[1] = (uint8_t)nCmdLen + 1;
    m_Linkbuffer_Out[2] = (uint8_t)nCMD & 0xff;

    if(nCmdLen>0 && pCMD != 0)
        memcpy(m_Linkbuffer_Out + 3, pCMD, nCmdLen);

    n_Len = nCmdLen + 3;

    SendData(m_Linkbuffer_Out, n_Len);
}

void ResponseDeviceDebugMessage(uint8_t *buf,uint8_t len)
{

    GenCommand(DEBUG_MSG_RESPONE_CMD, buf, len);

}

void ResponseACK(uint8_t CMD,uint8_t bSuccess)
{
    uint8_t ACK_CMD_BUF[2];
    ACK_CMD_BUF[0]=CMD;
    ACK_CMD_BUF[1]=bSuccess;
    GenCommand(ACK_CMD, ACK_CMD_BUF, 2);
}

void responeResetSuccess()
{
    ResponseACK(0xAA,CMD_SUCCESS);
}

void enterUpgrade()
{
	bEnterUpgradeMode=true;
	if(mem_flash_erase(INFO_ADDRESS,true))
    {
        ResponseACK(0xB1,CMD_SUCCESS);
    }
    else
    {
        ResponseACK(0xB1,CMD_FAIL);
    }
}

void getMode()
{
    uint8_t RESPONSE_CMD_BUF[1];

    RESPONSE_CMD_BUF[0]=INAPP;

    GenCommand(0xB0, RESPONSE_CMD_BUF, 1);
}

void responeVersion()
{
	uint8_t RESPONSE_CMD_BUF[2];

	RESPONSE_CMD_BUF[0]=(version&0xff00)>>8;
	RESPONSE_CMD_BUF[1]=(version&0xff);
	GenCommand(GET_FW_VERSION_RESPONSE_CMD, RESPONSE_CMD_BUF, 2);
}

void responeSensirion()
{
	int32_t temperature, humidity;
	uint8_t RESPONSE_CMD_BUF[8];

	getTemperatureAndHumidity(&temperature,&humidity);

	RESPONSE_CMD_BUF[0]=(temperature&0xff000000)>>24;
	RESPONSE_CMD_BUF[1]=(temperature&0x00ff0000)>>16;
	RESPONSE_CMD_BUF[2]=(temperature&0x0000ff00)>>8;
	RESPONSE_CMD_BUF[3]=(temperature&0x000000ff);

	RESPONSE_CMD_BUF[4]=(humidity&0xff000000)>>24;
	RESPONSE_CMD_BUF[5]=(humidity&0x00ff0000)>>16;
	RESPONSE_CMD_BUF[6]=(humidity&0x0000ff00)>>8;
	RESPONSE_CMD_BUF[7]=(humidity&0x000000ff);

	GenCommand(GET_TH_RESPONSE_CMD, RESPONSE_CMD_BUF, 8);
}

void responeDeviceID()
{

  //uint8_t RESPONSE_CMD_BUF[20];

  //getDeviceID(RESPONSE_CMD_BUF);

  //GenCommand(0x04, RESPONSE_CMD_BUF,6);
}

int8_t processWritePage(uint8_t *buf)
{
    uint8_t blockID=buf[0];
    uint8_t numberOfByte=0;
    uint8_t pageEESize=0;
    int8_t result=0;

    if(blockID==BLOCK_ID_PAGE0)
    {
	    numberOfByte=PAGE0_REG_MAX_SIZE;
	    pageEESize=PAGE0_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE2)
    {
	    numberOfByte=PAGE2_REG_MAX_SIZE;
	    pageEESize=PAGE2_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE3)
    {
	    numberOfByte=PAGE3_REG_MAX_SIZE;
	    pageEESize=PAGE3_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE4)
    {
	    numberOfByte=PAGE4_REG_MAX_SIZE;
	    pageEESize=PAGE4_SIZE;
    }
    else if(blockID==BLOCK_ID_TBD1)
    {
	    numberOfByte=TBD1_SIZE;
	    pageEESize=TBD1_SIZE;
    }
    else if(blockID==BLOCK_ID_TBD2)
    {
	    numberOfByte=TBD2_SIZE;
        pageEESize=TBD2_SIZE;
    }
    else if(blockID==BLOCK_ID_INFO1)
    {
	    numberOfByte=INFO1_SIZE;
        pageEESize=INFO1_SIZE;
    }
    else if(blockID==BLOCK_ID_INFO2)
    {
	    numberOfByte=INFO2_SIZE;
        pageEESize=INFO2_SIZE;
    }

    result=writePage(blockID,&buf[1],numberOfByte,pageEESize);

    return result;
}

void responsePagefromEEprom(uint8_t blockID)
{
    uint8_t pageBuf[129];
    uint8_t RESPONSE_CMD_BUF[150];
    uint16_t len=0;
    uint8_t pageEESize=0;
    uint8_t pageBufSize=0;

    memset(pageBuf,0x00,128);

    if(blockID==BLOCK_ID_PAGE0)
    {
	    pageEESize=PAGE0_SIZE;
	    pageBufSize=PAGE0_REG_MAX_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE2)
    {
	    pageEESize=PAGE2_SIZE;
	    pageBufSize=PAGE2_REG_MAX_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE3)
    {
	    pageEESize=PAGE3_SIZE;
	    pageBufSize=PAGE3_REG_MAX_SIZE;
    }
    else if(blockID==BLOCK_ID_PAGE4)
    {
	    pageEESize=PAGE4_SIZE;
	    pageBufSize=PAGE4_REG_MAX_SIZE;
    }
    else if(blockID==BLOCK_ID_TBD1)
    {
	    pageEESize=TBD1_SIZE;
	    pageBufSize=TBD1_SIZE;
    }
    else if(blockID==BLOCK_ID_TBD2)
    {
        pageEESize=TBD2_SIZE;
        pageBufSize=TBD2_SIZE;
    }
    else if(blockID==BLOCK_ID_INFO1)
    {
  	    pageEESize=INFO1_SIZE;
  	    pageBufSize=INFO1_SIZE;
    }
    else if(blockID==BLOCK_ID_INFO2)
    {
  	    pageEESize=INFO2_SIZE;
  	    pageBufSize=INFO2_SIZE;
    }

    getPageFromEEprom(blockID,pageBuf,pageEESize);
    RESPONSE_CMD_BUF[0]=blockID;
    memcpy(&RESPONSE_CMD_BUF[1],pageBuf,pageBufSize+1);
    len=pageBufSize+2;

    GenCommand(READ_EEPROM_BLOCK_RESPONSE_CMD, RESPONSE_CMD_BUF, len);
}

void responseEEpromOneByteRead(uint16_t addr)
{
	uint8_t RESPONSE_CMD_BUF[1];

	FIT_EE_Read(addr,RESPONSE_CMD_BUF,1);

	GenCommand(READ_EEPROM_RESPONSE_CMD, RESPONSE_CMD_BUF, 1);
}

void responsePagefromPCMD3140(uint8_t page)
{
    uint8_t pageBuf[128];
    uint8_t RESPONSE_CMD_BUF[128];
    uint16_t len=0;
    uint8_t numberOfReg=0;
    uint8_t checksum=0;

    memset(pageBuf,0x00,128);

    if(page==0)
    {
	    //dumpPage0();
	    dumpPage(0);
	    numberOfReg=PAGE0_REG_MAX_SIZE;
    }
    else if(page==1)
    {
	    //dumpPage1();
    	dumpPage(1);
	    numberOfReg=PAGE1_REG_MAX_SIZE;
    }
    else if(page==2)
    {
	    //dumpPage2();
	    dumpPage(2);
	    numberOfReg=PAGE3_REG_MAX_SIZE;
    }
    else if(page==3)
    {
	    //dumpPage3();
	    dumpPage(3);
	    numberOfReg=PAGE3_REG_MAX_SIZE;
    }
    else if(page==4)
    {
	    //dumpPage4();
	    dumpPage(4);
	    numberOfReg=PAGE4_REG_MAX_SIZE;
    }

    getPageFromSRAM(page,pageBuf,numberOfReg);
    checksum= calculateChecksum(pageBuf,numberOfReg);
    RESPONSE_CMD_BUF[0]=page;
    memcpy(&RESPONSE_CMD_BUF[1],pageBuf,numberOfReg);

    RESPONSE_CMD_BUF[numberOfReg+1]=checksum;
    len=numberOfReg+2;

    GenCommand(READ_PCMD_BLOCK_RESPONSE_CMD, RESPONSE_CMD_BUF, len);
}

void responeReadReg(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[2];

	RESPONSE_CMD_BUF[0]=cmd[0];
	RESPONSE_CMD_BUF[1]=readRegFromPCDM3140(cmd);

	GenCommand(READ_PCMD_REG_RESPONSE_CMD, RESPONSE_CMD_BUF, 2);
}

void responeReadRegWithPageParameters(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[3];

	RESPONSE_CMD_BUF[0]=cmd[0];
	RESPONSE_CMD_BUF[1]=cmd[1];
	RESPONSE_CMD_BUF[2]=readRegFromPCDM3140WithPageParameters(cmd);

	GenCommand(READ_PCMD_REG_WITH_PAGE_PARAMETERS_RESPONSE_CMD, RESPONSE_CMD_BUF, 3);
}

void responeReadModelNumber(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_MODE_NUMBER_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readModelNumber(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_MODEL_NUMBER_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_MODE_NUMBER_SIZE+1);
}

void responeReadVersionLetter(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_VERSION_LETTER_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readVersionLetter(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_VERSION_LETTER_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_VERSION_LETTER_SIZE+1);
}

void responeReadSerialNumber(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_SERIAL_NUMBER_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readSerialNumber(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_SERIAL_NUMBER_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_SERIAL_NUMBER_SIZE+1);
}

void responeReadSensitivity(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_SENSITIVITY_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readSensitivity(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_SENSITIVITY_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_SENSITIVITY_SIZE+1);
}

void responeReadUnitsCode(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_UNITS_CODE_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readUnitsCode(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_UNITS_CODE_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_UNITS_CODE_SIZE+1);
}

void responeReadReferenceFrequency(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_REFERENCE_FREQUENCY_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readReferenceFrequency(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_REFERENCE_FREQUENCY_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_REFERENCE_FREQUENCY_SIZE+1);
}

void responeReadFrequencyRangeMin(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_FREQUENCY_RANG_MIN_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readFrequencyRangeMin(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_FREQUENCY_RANG_MIN_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_FREQUENCY_RANG_MIN_SIZE+1);
}

void responeReadFrequencyRangeMax(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE1_FREQUENCY_RANG_MAX_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readFrequencyRangeMax(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_FREQUENCY_RANG_MAX_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE1_FREQUENCY_RANG_MAX_SIZE+1);
}

void responeReadChannelAssignment(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[CARTRIDGE2_CHANNEL_ASSIGNMEN_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readChannelAssignment(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_CHANNEL_ASSIGNMEN_RESPONSE_CMD, RESPONSE_CMD_BUF,CARTRIDGE2_CHANNEL_ASSIGNMEN_SIZE+1);
}

void responeReadDigitalInterfaceType(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_DIGITAL_INTERFACE_TYPE_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readDigitalInterfaceType(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_DIGITAL_INTERFACE_TYPE_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_DIGITAL_INTERFACE_TYPE_SIZE+1);
}

void responeReadBitClockFrequency(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_BIT_CLOCK_FREQUENCY_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readBitClockFrequency(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_BIT_CLOCK_FREQUENCY_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_BIT_CLOCK_FREQUENCY_SIZE+1);
}

void responeReadWordLength(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_WORD_LENGTH_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readWordLength(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_WORD_LENGTH_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_WORD_LENGTH_SIZE+1);
}

void responeReadSampleRate(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_SAMPLE_RATE_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readSampleRate(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_SAMPLE_RATE_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_SAMPLE_RATE_SIZE+1);
}

void responeReadCalibrationDate(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_CALIBRATION_DATE_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readCalibrationDate(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_CALIBRATION_DATE_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_CALIBRATION_DATE_SIZE+1);
}

void responeReadManufacturerID(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_MANUFACTUER_ID_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readManufacturerID(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_MANUFACTUER_ID_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_MANUFACTUER_ID_SIZE+1);
}

void responeReadFirmwareVersion(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_FW_VERSION_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readFirmwareVersion(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_FW_VERSION_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_FW_VERSION_SIZE+1);
}

void responeReadNotes(uint8_t *cmd)
{
	uint8_t RESPONSE_CMD_BUF[SYSTEM_NOTE_SIZE+1];

	RESPONSE_CMD_BUF[0]=cmd[0];
	readNotes(cmd[0],&RESPONSE_CMD_BUF[1]);

	GenCommand(READ_NOTE_RESPONSE_CMD, RESPONSE_CMD_BUF,SYSTEM_NOTE_SIZE+1);
}

bool enableFWLog(uint8_t bEnable)
{

	if(readEnableDebugMsgFlag()!=bEnable)
	{
	    if(writeEnableDebugMsgFlag(bEnable))
	    {
		    if(bEnable==0x01)
		    {
			    bShowDebugMSG=true;
		    }
		    else
		    {
			    bShowDebugMSG=false;
		    }
		    return true;
	    }
	}

	return false;
}

uint8_t ProcessPacket()
{
    uint8_t ret = 0;

    switch(RCV_PACKET[2])
    {
        case GET_FW_VERSION_CMD:
        responeVersion();
        break;
        case GET_TH_CMD:
        responeSensirion();
        break;
        case WRITE_EEPROM_BLOCK_CMD:
        {
			setLedNotify(LED_NOTIFY_TYPE_PEOGRAMMING);
			uint8_t result=processWritePage((uint8_t*)&RCV_PACKET[3]);
			if(result==2)
			{
				ResponseACK(WRITE_EEPROM_BLOCK_CMD,CMD_CHECKSUMERROR);
			}
			else if(result==1)
			{
				ResponseACK(WRITE_EEPROM_BLOCK_CMD,CMD_FAIL);
			}
			else
			{
				ResponseACK(WRITE_EEPROM_BLOCK_CMD,RCV_PACKET[3]+3);
			}
			setLedNotify(LED_NOTIFY_TYPE_NONE);
        }
        break;
        case READ_EEPROM_BLOCK_CMD:
        responsePagefromEEprom(RCV_PACKET[3]);
        break;
        case WRITE_EEPROM_CMD:
        {
            uint16_t addr = RCV_PACKET[3]<<8|RCV_PACKET[4];

            if(commandWriteOneByteToEE(addr,&RCV_PACKET[5])!=0)
            {
        	    ResponseACK(WRITE_EEPROM_CMD,CMD_FAIL);
            }
            else
            {
        	    ResponseACK(WRITE_EEPROM_CMD,CMD_SUCCESS);
            }
        }
		break;
        case READ_EEPROM_CMD:
        {
            uint16_t addr = RCV_PACKET[3]<<8|RCV_PACKET[4];
            responseEEpromOneByteRead(addr);
        }
        break;
        case WRITE_PCMD_BLOCK_CMD:
        {
			setLedNotify(LED_NOTIFY_TYPE_PEOGRAMMING);
			uint8_t result=writePCMD3140Page(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[1]-1,false);
			if(result==2)
			{
				ResponseACK(WRITE_PCMD_BLOCK_CMD,CMD_CHECKSUMERROR);
			}
			else if(result==1)
			{
				ResponseACK(WRITE_PCMD_BLOCK_CMD,CMD_FAIL);
			}
			else
			{
				ResponseACK(WRITE_PCMD_BLOCK_CMD,CMD_SUCCESS);
			}
			setLedNotify(LED_NOTIFY_TYPE_NONE);
        }
        break;
        case READ_PCMD_BLOCK_CMD:
        responsePagefromPCMD3140(RCV_PACKET[3]);
        break;
        case WRITE_PCMD_REG_CMD:
        if(writeRegToPCDM3140(&RCV_PACKET[3]))
		{
			ResponseACK(WRITE_PCMD_REG_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_PCMD_REG_CMD,CMD_FAIL);
		}
        break;
        case READ_PCMD_REG_CMD:
        responeReadReg(&RCV_PACKET[3]);
		break;
        case WRITE_PCMD_REG_WITH_PAGE_PARAMETERS_CMD:
		if(writeRegToPCDM3140WithPageParameters(&RCV_PACKET[3]))
		{
			ResponseACK(WRITE_PCMD_REG_WITH_PAGE_PARAMETERS_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_PCMD_REG_WITH_PAGE_PARAMETERS_CMD,CMD_FAIL);
		}
		break;
        case READ_PCMD_REG_WITH_PAGE_PARAMETERS_CMD:
        responeReadRegWithPageParameters(&RCV_PACKET[3]);
        break;
#if 0
        case WRITE_MODEL_NUMBER_CMD:
        if(writeModelNumber(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
        {
        	ResponseACK(WRITE_MODEL_NUMBER_CMD,CMD_SUCCESS);
        }
        else
        {
        	ResponseACK(WRITE_MODEL_NUMBER_CMD,CMD_FAIL);
        }
        break;
        case READ_MODEL_NUMBER_CMD:
        responeReadModelNumber(&RCV_PACKET[3]);
        break;
        case WRITE_VERSION_LETTER_CMD:
		if(writeVersionLetter(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_VERSION_LETTER_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_VERSION_LETTER_CMD,CMD_FAIL);
		}
		break;
        case READ_VERSION_LETTER_CMD:
		responeReadVersionLetter(&RCV_PACKET[3]);
		break;
        case WRITE_SERIAL_NUMBER_CMD:
		if(writeSerialNumber(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_SERIAL_NUMBER_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_SERIAL_NUMBER_CMD,CMD_FAIL);
		}
		break;
        case READ_SERIAL_NUMBER_CMD:
        responeReadSerialNumber(&RCV_PACKET[3]);
		break;
        case WRITE_SENSITIVITY_CMD:
		if(writeSensitivity(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_SENSITIVITY_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_SENSITIVITY_CMD,CMD_FAIL);
		}
		break;
        case READ_SENSITIVITY_CMD:
        responeReadSensitivity(&RCV_PACKET[3]);
		break;
        case WRITE_REFERENCE_FREQUENCY_CMD:
		if(writeReferenceFrequency(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_REFERENCE_FREQUENCY_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_REFERENCE_FREQUENCY_CMD,CMD_FAIL);
		}
		break;
		case READ_REFERENCE_FREQUENCY_CMD:
		responeReadReferenceFrequency(&RCV_PACKET[3]);
		break;
		case WRITE_UNITS_CODE_CMD:
		if(writeUnitsCode(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_UNITS_CODE_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_UNITS_CODE_CMD,CMD_FAIL);
		}
		break;
		case READ_UNITS_CODE_CMD:
		responeReadUnitsCode(&RCV_PACKET[3]);
		break;
		case WRITE_FREQUENCY_RANG_MIN_CMD:
		if(writeFrequencyRangeMin(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_FREQUENCY_RANG_MIN_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_FREQUENCY_RANG_MIN_CMD,CMD_FAIL);
		}
		break;
		case READ_FREQUENCY_RANG_MIN_CMD:
		responeReadFrequencyRangeMin(&RCV_PACKET[3]);
		break;
		case WRITE_FREQUENCY_RANG_MAX_CMD:
		if(writeFrequencyRangeMax(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_FREQUENCY_RANG_MAX_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_FREQUENCY_RANG_MAX_CMD,CMD_FAIL);
		}
		break;
		case READ_FREQUENCY_RANG_MAX_CMD:
		responeReadFrequencyRangeMax(&RCV_PACKET[3]);
		break;
		case WRITE_CHANNEL_ASSIGNMEN_CMD:
		if(writeChannelAssignment(RCV_PACKET[3],&RCV_PACKET[4],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_CHANNEL_ASSIGNMEN_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_CHANNEL_ASSIGNMEN_CMD,CMD_FAIL);
		}
		break;
		case READ_CHANNEL_ASSIGNMEN_CMD:
		responeReadChannelAssignment(&RCV_PACKET[3]);
		break;
		case WRITE_DIGITAL_INTERFACE_TYPE_CMD:
		if(writeDigitalInterfaceType(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_DIGITAL_INTERFACE_TYPE_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_DIGITAL_INTERFACE_TYPE_CMD,CMD_FAIL);
		}
		break;
		case READ_DIGITAL_INTERFACE_TYPE_CMD:
		responeReadDigitalInterfaceType(&RCV_PACKET[3]);
		break;
		case WRITE_BIT_CLOCK_FREQUENCY_CMD:
		if(writeBitClockFrequency(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_BIT_CLOCK_FREQUENCY_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_BIT_CLOCK_FREQUENCY_CMD,CMD_FAIL);
		}
		break;
		case READ_BIT_CLOCK_FREQUENCY_CMD:
		responeReadBitClockFrequency(&RCV_PACKET[3]);
		break;
		case WRITE_WORD_LENGTH_CMD:
		if(writeWordLength(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_WORD_LENGTH_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_WORD_LENGTH_CMD,CMD_FAIL);
		}
		break;
		case READ_WORD_LENGTH_CMD:
		responeReadWordLength(&RCV_PACKET[3]);
		break;
		case WRITE_SAMPLE_RATE_CMD:
		if(writeSampleRate(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_SAMPLE_RATE_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_SAMPLE_RATE_CMD,CMD_FAIL);
		}
		break;
		case READ_SAMPLE_RATE_CMD:
		responeReadSampleRate(&RCV_PACKET[3]);
		break;
		case WRITE_CALIBRATION_DATE_CMD:
		if(writeCalibrationDate(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_CALIBRATION_DATE_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_CALIBRATION_DATE_CMD,CMD_FAIL);
		}
		break;
		case READ_CALIBRATION_DATE_CMD:
		responeReadCalibrationDate(&RCV_PACKET[3]);
		break;
		case WRITE_MANUFACTUER_ID_CMD:
		if(writeManufacturerID(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_MANUFACTUER_ID_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_MANUFACTUER_ID_CMD,CMD_FAIL);
		}
		break;
		case READ_MANUFACTUER_ID_CMD:
		responeReadManufacturerID(&RCV_PACKET[3]);
		break;
		case WRITE_FW_VERSION_CMD:
		if(writeFirmwareVersion(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_FW_VERSION_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_FW_VERSION_CMD,CMD_FAIL);
		}
		break;
		case READ_FW_VERSION_CMD:
		responeReadFirmwareVersion(&RCV_PACKET[3]);
		break;
		case WRITE_NOTE_CMD:
		if(writeNotes(&RCV_PACKET[3],RCV_PACKET[PAYLOADLEN]-2))
		{
			ResponseACK(WRITE_NOTE_CMD,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(WRITE_NOTE_CMD,CMD_FAIL);
		}
		break;
		case READ_NOTE_CMD:
		responeReadNotes(&RCV_PACKET[3]);
		break;
#endif
        case 0xA0:
        //testEEprom();
        break;
        case 0xAA:
		NVIC_SystemReset();
		break;
		case 0xB0:
		getMode();
		break;
		case 0xB1:
		enterUpgrade();
		break;
		case 0xB5:
		if(UPGRADE_INIT((uint8_t*)&RCV_PACKET[3]))
		{
			ResponseACK(0xB2,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(0xB2,CMD_FAIL);
		}
		break;
		case 0xB6:
		UPGRADE_START((uint8_t*)&RCV_PACKET[3],RCV_PACKET[1]-1);
		ResponseACK(0xB3,CMD_SUCCESS);
		break;
		case 0xB7:
		if(UPGRADE_END((uint8_t*)&RCV_PACKET[3],RCV_PACKET[1]-1))
		{
			ResponseACK(0xB4,CMD_SUCCESS);
		}
		else
		{
			ResponseACK(0xB4,CMD_FAIL);
		}
		break;
		case 0xF0:
		enableFWLog(RCV_PACKET[3]);
		break;
        case 0xFF:
        break;
        default:
        FT_printf("fail unknow command\r\n");
        break;
    }

    return ret;
}

uint8_t PreparePacketToParse()
{
    uint8_t result=0;
    while(DataIsValid())
    {

    	uint8_t Read_Byte=Read_Data();
    	RCV_PACKET[rcv_index]= Read_Byte;
    	//cbWrite(&rb,(char*)&Write_Byte);

        if(RCV_PACKET[STARTBYTE]==0x55)
        {
        	 if(RCV_PACKET[PAYLOADLEN]>=RCV_CMD_MAX_LEN)
			{
        		rcv_index=0;
				break;
			}

        	if(RCV_PACKET[PAYLOADLEN]==rcv_index-1)
            {
                result=1;
                rcv_index=0;
                return result;
            }
            rcv_index++;
        }
        else
        {
        	ftm_get_conmmand(Read_Byte);
        }

        //if(rcv_index>=RCV_CMD_MAX_LEN)
        //{
        //     rcv_index=0;
        //     break;
        //}
    }

    return result;
}


void proccess_rxbuf()
{

	if(PreparePacketToParse()==0x01)
    {
    	ftm_clear_cmd_line();
    	ProcessPacket();
    }

}

void uartErrorCallback()
{
    __IO uint32_t isr_reg;

    /* Disable USARTx_IRQn */
    NVIC_DisableIRQ(USART2_IRQn);

    /* Error handling example :
       - Read USART ISR register to identify flag that leads to IT raising
	   - Perform corresponding error handling treatment according to flag
    */
    isr_reg = LL_USART_ReadReg(USART2, ISR);
    FT_printf("usart2 error:%x\r\n",isr_reg);
    init_DebugMessage();
}

void processDebugRX()
{
    //if (LL_LPUART_IsActiveFlag_RXNE(LPUART1) && (LL_LPUART_IsEnabledIT_RXNE(LPUART1) != RESET) )
    if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
    {
        //setIdleCounter(2000);
        //RXBuffer[rxbuf_index] = LL_LPUART_ReceiveData8(LPUART1);
        RXBuffer[rxbuf_index] = LL_USART_ReceiveData8(USART2);

        rxbuf_index++;
        if(rxbuf_index==512)
          rxbuf_index=0;
    }
    //else
    //{
    // 	uartErrorCallback();
    //}
}
