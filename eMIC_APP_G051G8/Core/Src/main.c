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
#include "FIT_DebugMessage.h"
#include "FIT_Flash.h"
#include "sht4x.h"
#include "FIT_PCMD3140.h"
#include "FIT_M24C08_EE.h"
#include "FIT_SysTick.h"
#include "FIT_LED.h"
#include "FIT_sensirion_hw_i2c_implementation.h"
#include "FIT_eeprom_pcmd3140_hw_i2c_implemention.h"
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
const char APP[]="FIT_V1APP_VER_R0003_BETA_00@";
int version=0;
int beta=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
    for(int addr=betaOffset;addr<betaOffset+2;addr++)
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
        FT_printf("Version:R%02d\r\n",version);
    else
        FT_printf("Version:R%02d Beta%02d\r\n",version,beta);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GetVersion(&version,&beta,15,25);
  eeprom_pcmd3140_i2c_init();
  init_DebugMessage();
  sensirion_i2c_init();

  uint32_t serial;

  sht4x_read_serial(&serial);

  FT_printf("sht4x serial number:%lx!!\r\n",serial);

  initEEPROM();
  init_PCMD3140();
  initLED();
  initSysTick();

  showVersion();

  FT_printf("book ok!!\r\n");

  responeResetSuccess();

  while (1)
  {
	  proccess_rxbuf();
	  processPCMD3140();
	  //processLED();
  }

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
