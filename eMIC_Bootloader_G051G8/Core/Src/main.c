/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FIT_Upgrade.h"
#include "FIT_FLASH.h"
#include "FIT_DebugMessage.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
const char APP[]="FIT_V1BL1_VER_R0001_BETA_00@";
int version=0;
int beta=0;

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t bootAppaddress=APPLICATION_ADDRESS;
uint8_t *BootInfo = (uint8_t*)INFO_ADDRESS;

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void GetVersion(int *ver,int *beta,uint8_t verOffset,uint8_t betaOffset)
{
    int Counter=1;

    for(int addr=verOffset;addr<verOffset+4;addr++)
    {
        if(APP[addr]>=0x30&&APP[addr]<=0x39)
        {
            *ver+=(APP[addr]%0x30)*(1000/Counter);
        }

        Counter*=10;
    }
    Counter=1;
    for(int addr=betaOffset+10;addr<betaOffset+2;addr++)
    {
        if(APP[addr]>=0x30&&APP[addr]<=0x39)
        {
            *beta+=(APP[addr]%0x30)*(10/Counter);
        }
        Counter*=10;
    }
}

void showVersion()
{
    if(beta==0)
    	FT_printf("BL Version:R%02d\r\n",version);
    else
    	FT_printf("BL Version:R%02d Beta%02d\r\n",version,beta);
}

void iap_load_app(uint32_t appAddr)
{
	FT_printf("boot addr : 0x%lx\n",appAddr);
	FT_printf("first word : 0x%lx\n",(*(uint32_t*)appAddr));
    /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
    if (((*(__IO uint32_t*)appAddr) & 0x2FFE0000 ) == 0x20000000)
    {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (appAddr + 4);
        Jump_To_Application = (pFunction) JumpAddress;

        __set_MSP(*(__IO uint32_t*) appAddr);

        /* Jump to application */
        Jump_To_Application();
    }
}

bool checkBootInfo()
{
	FT_printf("%x %x %x %x %x\r\n",BootInfo[0],BootInfo[1],BootInfo[128],BootInfo[129],BootInfo[130]);
	if(BootInfo[0]==0x01&&BootInfo[128]==0x46&&BootInfo[129]==0x49&&BootInfo[130]==0x54&&BootInfo[256]==0x46&&BootInfo[257]==0x49&&BootInfo[258]==0x54)
    {
        bootAppaddress=(BootInfo[1]<<24)|(BootInfo[2]<<16)|(BootInfo[3]<<8)|BootInfo[4];
        uint8_t *image = (uint8_t*)APPLICATION_ADDRESS;
        uint32_t flashImageChecksum=0;
        uint32_t BootInfoChecksum=(BootInfo[284]<<24)|(BootInfo[285]<<16)|(BootInfo[286]<<8)|BootInfo[287];
        for(uint32_t i=0;i<(INFO_ADDRESS-bootAppaddress);i++)
        {
            flashImageChecksum+=image[i];
        }
        FT_printf("Checksum:%x %x\r\n",flashImageChecksum,BootInfoChecksum);
        if(flashImageChecksum==BootInfoChecksum)
            return true;
    }
    return false;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, 3);

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    GetVersion(&version,&beta,15,25);
    init_DebugMessage();
    showVersion();
    if(checkBootInfo())
    {
	    iap_load_app(bootAppaddress);
    }
    else
    {
	    FT_printf("Enter upgrade mode!!\r\n");
	    responeResetSuccess();
    }

    //mem_flash_erase(0x08002800,true);
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
	    proccess_rxbuf();
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    }

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the HSI */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
    {
    }

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(16000000);
    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(16000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
