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
#include "Command.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

uint8_t RXBuffer[192];
uint8_t RCV_PACKET[192];
uint8_t rxbuf_index=0;
uint8_t temprxbuf_index=0;
uint8_t rcv_index=0;
extern int version;


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
    LL_USART_EnableIT_ERROR(USART2);

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

void FT_printf(const char *fmt, ...)
{
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
        if(temprxbuf_index==64)
            temprxbuf_index=0;
        return data;
    }
    return 0;
}


void SendData(uint8_t *data,uint8_t dataLen)
{
	put_s((char*)data,dataLen);
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

void ResponseACK(uint8_t CMD,uint8_t bSuccess)
{
    uint8_t ACK_CMD_BUF[2];
    ACK_CMD_BUF[0]=CMD;
    ACK_CMD_BUF[1]=bSuccess;
    GenCommand(ACK_CMD, ACK_CMD_BUF, 2);
}

void ResponseDeviceDebugMessage(uint8_t *buf,uint8_t len)
{

    GenCommand(DEBUG_MSG_RESPONE_CMD, buf, len);

}

void responeResetSuccess()
{
    ResponseACK(REBOOT_CMD,CMD_SUCCESS);
}

void enterUpgrade()
{
    if(mem_flash_erase(INFO_ADDRESS,true))
    {
        ResponseACK(0xB1,CMD_SUCCESS);
    }
    else
    {
        ResponseACK(OTA_ENTER_UPGRADE_MODE_CMD,CMD_FAIL);
    }
}

void getMode()
{
    uint8_t RESPONSE_CMD_BUF[1];

    RESPONSE_CMD_BUF[0]=INBOOTLOADER;

    GenCommand(GATMODE_CMD, RESPONSE_CMD_BUF, 1);
}

void responeVersion()
{
	uint8_t RESPONSE_CMD_BUF[2];

	RESPONSE_CMD_BUF[0]=0xff;
	RESPONSE_CMD_BUF[1]=(version&0xff);
	GenCommand(GET_FW_VERSION_RESPONSE_CMD, RESPONSE_CMD_BUF, 2);
}


uint8_t ProcessPacket()
{
    uint8_t ret = 0;

    switch(RCV_PACKET[2])
    {
        case GET_FW_VERSION_CMD:
        responeVersion();
        break;
        case REBOOT_CMD:
        NVIC_SystemReset();
        break;
        case GATMODE_CMD:
        getMode();
        break;
        case OTA_ENTER_UPGRADE_MODE_CMD:
        enterUpgrade();
        break;
        case OTA_UPGRADE_INIT_CMD:
        if(UPGRADE_INIT((uint8_t*)&RCV_PACKET[3]))
        {
            ResponseACK(0xB2,CMD_SUCCESS);
        }
        else
        {
            ResponseACK(0xB2,CMD_FAIL);
        }
        break;
        case OTA_UPGRADE_START_CMD:
        if(UPGRADE_START((uint8_t*)&RCV_PACKET[3],RCV_PACKET[1]-1))
        {
            ResponseACK(0xB3,CMD_SUCCESS);
        }
        else
        {
        	ResponseACK(0xB4,CMD_FAIL);
        }
        break;
        case OTA_UPGRADE_END_CMD:
        if(UPGRADE_END((uint8_t*)&RCV_PACKET[3],RCV_PACKET[1]-1))
        {
            ResponseACK(0xB4,CMD_SUCCESS);
        }
        else
            ResponseACK(0xB4,CMD_FAIL);
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
        RCV_PACKET[rcv_index]= Read_Data();
        if(RCV_PACKET[0]==0x55)
        {
            if(RCV_PACKET[1]==rcv_index-1)
            {
                result=1;
                rcv_index=0;
                return result;
            }
            rcv_index++;
        }

        if(rcv_index>=RCV_CMD_MAX_LEN)
        {
            rcv_index=0;
            break;
        }
    }

    return result;
}


void proccess_rxbuf()
{
    if(PreparePacketToParse()==0x01)
    {
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
        if(rxbuf_index==64)
          rxbuf_index=0;
    }
    else
    {
    	uartErrorCallback();
    }
}
