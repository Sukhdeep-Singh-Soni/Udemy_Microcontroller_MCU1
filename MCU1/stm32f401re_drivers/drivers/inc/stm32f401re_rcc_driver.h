/*
 * stm32f401re_rcc_driver.h
 *
 *  Created on: Dec 15, 2024
 *      Author: deep
 */

#ifndef INC_STM32F401RE_RCC_DRIVER_H_
#define INC_STM32F401RE_RCC_DRIVER_H_

#include "stm32f401re.h"

uint32_t RCC_GetPclk1Freq(void);
uint32_t RCC_GetPLLClock(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F401RE_RCC_DRIVER_H_ */
