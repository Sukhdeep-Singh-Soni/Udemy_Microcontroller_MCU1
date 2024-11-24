/*
 * stm32f401re_gpio_driver.c
 *
 *  Created on: Nov 24, 2024
 *      Author: deep
 */

#include "stm32f401re_gpio_driver.h"

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_PCLKCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_CLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_CLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_CLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_CLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_CLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_CLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_CLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_CLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_CLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_CLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_CLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_CLK_DI();
		}
	}
}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Value, uint8_t PinNumber) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t IRQNumber, uint8_t IRQPriority) {

}

/*
 * @fn		-	GPIO_PCLKCtrl
 * @brief	-	enables or disables the gpio peripheral clock
 * @param	-	base address of gpiox peripheral
 * @param	-	ENABLE or DISABLE macros
 * @ret		-	none
 * @note	-	none
 * */
void IRQConfig(uint8_t PinNumber) {

}

