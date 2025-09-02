/*
 * systick.c
 *
 *  Created on: May 17, 2021
 *      Author: fit0354
 */
#include "main.h"
#include "FIT_LED.h"

extern uint32_t SystemCoreClock;
uint32_t SysTickCount=0;

void initSysTick()
{
    SysTick_Config(SystemCoreClock / 1000);
}

void resetSysTick()
{
    SysTickCount=0;
}

void addSysTick(uint32_t count)
{
    SysTickCount+=count;
}

uint32_t HAL_GetTick()
{
    return SysTickCount;
}

void SysTickCallback()
{
    SysTickCount++;
    processLED();
    //processSystemIdle();
    //incADCCount();
}
