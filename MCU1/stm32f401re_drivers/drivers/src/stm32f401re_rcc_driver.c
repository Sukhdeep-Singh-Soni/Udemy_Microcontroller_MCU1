/*
 * stm32f401re_rcc_driver.c
 *
 *  Created on: Dec 15, 2024
 *      Author: deep
 */

#include "stm32f401re_rcc_driver.h"

uint16_t AHB1_Prescalar[] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Presaclar[] = {2, 4, 8, 16};
uint8_t APB2_Presaclar[] = {2, 4, 8, 16};

/*
 * @fn		-	RCC_GetPclkFreq
 * @brief	-	get the system clock frequency
 * @param	-	none
 * @ret		-	none
 * @note	-	none
 * */
uint32_t RCC_GetPclk1Freq(void) {
	uint32_t cfgr, ahb1p, apb1p;
	uint32_t SystemClk, pclk;

	cfgr = ((RCC->CFGR >> 2) & 0x3);

	if(cfgr == 0) {
		/*HSI is used as system clock*/
		SystemClk = 16000000;
	} else if(cfgr == 1) {
		/*HSE is used as system clock*/
		/*in nucleo-f401re no external crystal is used*/
		SystemClk = 0;
	} else if(cfgr == 2) {
		/*PLL clock calculate*/
		SystemClk = RCC_GetPLLClock();
	}

	cfgr = ((RCC->CFGR >> 4) & 0xf);

	if(cfgr < 8) {
		ahb1p = 1;
	} else {
		ahb1p = AHB1_Prescalar[cfgr - 8];
	}

	cfgr = ((RCC->CFGR >> 10) & 0x7);

	if(cfgr < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_Presaclar[cfgr - 4];
	}

	pclk = ((SystemClk / ahb1p) / apb1p);

	return pclk;
}

uint32_t RCC_GetPLLClock(void) {
	/*not implemented yet*/
	return 0;
}

uint32_t RCC_GetPCLK2Value(void) {
	uint32_t cfgr, ahb1p, apb2p;
	uint32_t SystemClk, pclk2;

	cfgr = ((RCC->CFGR >> 2) & 0x3);

	if(cfgr == 0) {
		/*HSI is used as system clock*/
		SystemClk = 16000000;
	} else if(cfgr == 1) {
		/*HSE is used as system clock*/
		/*in nucleo-f401re no external crystal is used*/
		SystemClk = 0;
	} else if(cfgr == 2) {
		/*PLL clock calculate*/
		SystemClk = RCC_GetPLLClock();
	}

	cfgr = ((RCC->CFGR >> 4) & 0xf);

	if(cfgr < 8) {
		ahb1p = 1;
	} else {
		ahb1p = AHB1_Prescalar[cfgr - 8];
	}

	cfgr = ((RCC->CFGR >> 13) & 0x7);

	if(cfgr < 4) {
		apb2p = 1;
	} else {
		apb2p = APB2_Presaclar[cfgr - 4];
	}

	pclk2 = ((SystemClk / ahb1p) / apb2p);

	return pclk2;
}



