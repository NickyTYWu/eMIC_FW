/*
 * FIT_LED.c
 *
 *  Created on: Jun 24, 2025
 *      Author: fit0354
 */
#include "main.h"
#include "FIT_LED.h"
#include "FIT_DebugMessage.h"
#include "FIT_PCMD3140.h"
#include "FIT_SysTick.h"

bool bTubeLedOn=false;
bool bHubLedOn=false;

uint8_t ledActionType=0;
uint8_t ledNotifyType = 0x00;
uint8_t prvLedNotifyType = 0x00;

uint32_t blinkInterval1=0;
uint32_t blinkInterval2=0;

uint32_t ledBlinkCount=0;
int      ledBlinkTimes=0;
int      tempLedBlinkTimes=0;

void ResetLED_Group()
{
	LL_GPIO_ResetOutputPin(LED_TUBE_GPIO_Port, LED_TUBE_Pin);
	LL_GPIO_ResetOutputPin(LED_HUB_GPIO_Port, LED_HUB_Pin);

    bTubeLedOn=bHubLedOn=false;
}

void TurnOffTubeLED()
{
	LL_GPIO_ResetOutputPin(LED_TUBE_GPIO_Port, LED_TUBE_Pin);
	bTubeLedOn=false;
}

void TurnOffHubLED()
{
	LL_GPIO_ResetOutputPin(LED_HUB_GPIO_Port, LED_HUB_Pin);
	bHubLedOn=false;
}

void TurnOnTubeLED()
{
	LL_GPIO_SetOutputPin(LED_TUBE_GPIO_Port, LED_TUBE_Pin);
	bTubeLedOn=true;
}

void TurnOnHubLED()
{
	LL_GPIO_SetOutputPin(LED_HUB_GPIO_Port, LED_HUB_Pin);
	bHubLedOn=true;
}

void resetLEDParamater()
{
	ledActionType=LED_ACTION_TYPE_NONE;

	ledBlinkCount=0;
	tempLedBlinkTimes=0;
	blinkInterval1=0;
	blinkInterval2=0;
	ledBlinkTimes=0;
}

void setLEDBlinkingError(uint8_t actionType,uint32_t interval,uint32_t interval2,int times)
{
	ledActionType=actionType;

	ResetLED_Group();
	ledBlinkCount=HAL_GetTick();
	tempLedBlinkTimes=0;
	blinkInterval1=interval;
	blinkInterval2=interval2;
	ledBlinkTimes=times;

}

void setLEDBlinkingSlowly(uint8_t actionType,uint32_t interval,uint32_t interval2,int times)
{
	ledActionType=actionType;

	TurnOnTubeLED();
	TurnOnHubLED();
	ledBlinkCount=HAL_GetTick();
	tempLedBlinkTimes=0;
	blinkInterval1=interval;
	blinkInterval2=interval2;
	ledBlinkTimes=times;

}

void setLEDBlinkingProgramming(uint8_t actionType,uint32_t interval)
{
	ledActionType=actionType;

	ledBlinkCount=HAL_GetTick();
	blinkInterval1=interval;
}

uint8_t getLedNotifyType()
{
	return ledNotifyType;
}

void setLedNotify(uint8_t event)
{
	if(ledNotifyType!=event)
	{
		if(ledNotifyType==LED_NOTIFY_TYPE_PEOGRAMMING&&event!=LED_NOTIFY_TYPE_NONE)
			return;

		ledNotifyType=event;
	}
	else
	{
		return;
	}

	switch(ledNotifyType){
	    case LED_NOTIFY_TYPE_NONE:
	    resetLEDParamater();
		break;
	    case LED_NOTIFY_TYPE_PCMD_INIT_FAIL:
	    setLEDBlinkingSlowly(LED_ACTION_TYPE_BLINKING,500,3000,-1);
	    break;
	    case LED_NOTIFY_TYPE_PEOGRAMMING:
		setLEDBlinkingSlowly(LED_NOTIFY_TYPE_PEOGRAMMING,80,80,-1);
		break;
	    case LED_NOTIFY_TYPE_I2C_TIMEOUT:
	    setLEDBlinkingSlowly(LED_ACTION_TYPE_BLINKING,40,40,8);
	    break;
	    case LED_NOTIFY_TYPE_ASI_BUS_CLOCK_ERROR:
	    setLEDBlinkingError(LED_ACTION_TYPE_BLINKING_ERROR,1000,5000,2);
		break;
	    case LED_NOTIFY_TYPE_PLL_CLOCK_ERROR:
		setLEDBlinkingError(LED_ACTION_TYPE_BLINKING_ERROR,1000,5000,4);
		break;
	    case LED_NOTIFY_TYPE_ASI_INPUT_MIXING_SATURATION_ERROR:
		setLEDBlinkingError(LED_ACTION_TYPE_BLINKING_ERROR,1000,5000,6);
		break;
	    case LED_NOTIFY_TYPE_VAD_POWER_UP_DETECT:
		setLEDBlinkingError(LED_ACTION_TYPE_BLINKING_ERROR,1000,5000,8);
		break;
	    case LED_NOTIFY_TYPE_VAD_POWER_DOWN_DETECT:
		setLEDBlinkingError(LED_ACTION_TYPE_BLINKING_ERROR,1000,5000,10);
		break;
	}
}

void processLED()
{
    if(isPCMD3140DataCorrect()&&ledActionType==LED_ACTION_TYPE_NONE)
    {
    	if(!bTubeLedOn)
    		TurnOnTubeLED();

    	if(!bHubLedOn)
    		TurnOnHubLED();
    }
    else
    {
        if(ledActionType==LED_ACTION_TYPE_BLINKING)
        {
            if(bTubeLedOn||bHubLedOn)
            {
        	    if(HAL_GetTick()-ledBlinkCount>=blinkInterval1)
                {
        	    	ResetLED_Group();
        	    	ledBlinkCount = HAL_GetTick();
        	    	if(ledBlinkTimes>0)
        	    		tempLedBlinkTimes++;
                }
            }
            else
            {
            	if(HAL_GetTick()-ledBlinkCount>=blinkInterval2)
				{
            		TurnOnTubeLED();
            		TurnOnHubLED();
					ledBlinkCount = HAL_GetTick();
					if(ledBlinkTimes>0)
						tempLedBlinkTimes++;
				}
            }

            if(tempLedBlinkTimes>0)
            {
				if(tempLedBlinkTimes>=ledBlinkTimes)
				{
					ledActionType = LED_ACTION_TYPE_NONE;
					ledBlinkTimes=0;
					tempLedBlinkTimes=0;
					ledBlinkCount=0;
					blinkInterval1=0;
					blinkInterval2=0;
					ledNotifyType=LED_NOTIFY_TYPE_NONE;
					ResetLED_Group();
				}
            }

        }
        else if(ledActionType==LED_ACTION_TYPE_BLINKING_PROGRAMMING)
        {
        	if(HAL_GetTick()-ledBlinkCount>=blinkInterval1)
        	{

        		if(bTubeLedOn)
        			TurnOffTubeLED();
				else
					TurnOnTubeLED();

        		if(bHubLedOn)
        			TurnOffHubLED();
        		else
        			TurnOnHubLED();

        		ledBlinkCount=HAL_GetTick();
        	}
        }
        else if(ledActionType==LED_ACTION_TYPE_BLINKING_ERROR)
		{
        	if(tempLedBlinkTimes>=ledBlinkTimes)
			{
        		if(HAL_GetTick()-ledBlinkCount>=blinkInterval2)
        		{
        		    ledActionType = LED_ACTION_TYPE_NONE;
				    ledBlinkTimes=0;
				    tempLedBlinkTimes=0;
				    ledBlinkCount=0;
				    blinkInterval1=0;
				    blinkInterval2=0;
				    ledNotifyType=LED_NOTIFY_TYPE_NONE;
				    ResetLED_Group();
        		}
			}
        	else
        	{
				if(bTubeLedOn||bHubLedOn)
				{
					if(HAL_GetTick()-ledBlinkCount>=blinkInterval1)
					{
						ResetLED_Group();
						ledBlinkCount = HAL_GetTick();
						if(ledBlinkTimes>0)
							tempLedBlinkTimes++;
					}
				}
				else
				{
					if(HAL_GetTick()-ledBlinkCount>=blinkInterval1)
					{
						TurnOnTubeLED();
						TurnOnHubLED();
						ledBlinkCount = HAL_GetTick();
						if(ledBlinkTimes>0)
							tempLedBlinkTimes++;
					}
				}
        	}

		}
    }
}

void initLED()
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	LL_GPIO_ResetOutputPin(LED_TUBE_GPIO_Port, LED_TUBE_Pin);
	LL_GPIO_ResetOutputPin(LED_HUB_GPIO_Port, LED_HUB_Pin);

	/**/
	GPIO_InitStruct.Pin = LED_TUBE_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(LED_TUBE_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LED_HUB_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(LED_HUB_GPIO_Port, &GPIO_InitStruct);

	ResetLED_Group();

}
