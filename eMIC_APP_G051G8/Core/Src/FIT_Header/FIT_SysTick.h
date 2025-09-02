/*
 * systick.h
 *
 *  Created on: May 17, 2021
 *      Author: fit0354
 */

#ifndef FIT_COMMON_SYSTICK_H_
#define FIT_COMMON_SYSTICK_H_

void resetSysTick();
uint32_t HAL_GetTick();
void SysTickCallback();
void initSysTick();
void addSysTick(uint32_t count);
#endif /* FIT_COMMON_SYSTICK_H_ */
