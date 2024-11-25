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
 * @fn		-	GPIO_Init
 * @brief	-	initialize gpio pin with provided configuration given by user
 * 				in pGPIOHandle(user need to initialize this handle)
 * @param	-	pointer to GPIO_Handle_t structure(contains GPIO pin configuration)
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;
	/*1. configure the gpio mode*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //set
		temp = 0;
	} else {
		//interrupt mode
	}
	/*2. configure the output speed*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	/*3. configure pullup/pulldown*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	/*4. configure GPIO output type*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	/*5. configure GPIO alternate functionality*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint32_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8; /*index to goto*/
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8; /*position to shift to*/
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 * temp2)); //clear first
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp; //set alternate functionality
	}
}

/*
 * @fn		-	GPIO_DeInit
 * @brief	-	resets the specified gpio port in GPIO_RegDef_t param
 * @param	-	base address of gpiox peripheral
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_RESET_REG();
	} else if(pGPIOx == GPIOB) {
		GPIOB_RESET_REG();
	} else if(pGPIOx == GPIOC) {
		GPIOC_RESET_REG();
	} else if(pGPIOx == GPIOD) {
		GPIOD_RESET_REG();
	} else if(pGPIOx == GPIOE) {
		GPIOE_RESET_REG();
	} else if(pGPIOx == GPIOH) {
		GPIOH_RESET_REG();
	}
}

/*
 * @fn		-	GPIO_ReadFromInputPin
 * @brief	-	read from input data register of GPIO and returns the specified pin status
 * @param	-	base address of gpiox peripheral
 * @param	-	pin number to read status from
 * @ret		-	0 or 1
 * @note	-	none
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;

	value = (uint8_t)(((pGPIOx->IDR) >> PinNumber) & 0x00000001U);

	return value;
}

/*
 * @fn		-	GPIO_ReadFromInputPort
 * @brief	-	read the whole IDR value
 * @param	-	base address of gpiox peripheral
 * @ret		-	16-bit value returning the status of 0/1 for each pin0-pin15
 * @note	-	none
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*
 * @fn		-	GPIO_WriteToOutputPin
 * @brief	-	write to GPIO pin
 * @param	-	base address of gpiox peripheral
 * @param	-	value to write to pin 0 or 1
 * @param	-	gpio pin to write to
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Value, uint8_t PinNumber) {
	if(Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * @fn		-	GPIO_WriteToOutputPort
 * @brief	-	write to whole gpio output port
 * @param	-	base address of gpiox peripheral
 * @param	-	16-bit value to write to
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

		pGPIOx->ODR = (uint16_t)Value;
}

/*
 * @fn		-	GPIO_TogglePin
 * @brief	-	toggle the pin specified of gpio
 * @param	-	base address of gpiox peripheral
 * @param	-	pin numebr to toggle
 * @ret		-	none
 * @note	-	none
 * */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);
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

