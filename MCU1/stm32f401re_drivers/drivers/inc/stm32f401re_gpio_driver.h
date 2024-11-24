/*
 * stm32f401re_gpio_driver.h
 *
 *  Created on: Nov 24, 2024
 *      Author: deep
 */

#ifndef INC_STM32F401RE_GPIO_DRIVER_H_
#define INC_STM32F401RE_GPIO_DRIVER_H_

#include "stm32f401re.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIOPinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPd;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunc;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; /*This contains the base addr of GPIO peripheral to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*###############################################################################
 * 					API's provided by this driver
 *###############################################################################
 */

/*
 * Clock control api's
 * */
void GPIO_PCLKCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and DeInit API's
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write api's
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Value, uint8_t PinNumber);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/*
 * Interrupt configuration and Handling
 * */
void IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t IRQNumber, uint8_t IRQPriority);
void IRQConfig(uint8_t PinNumber);

#endif /* INC_STM32F401RE_GPIO_DRIVER_H_ */
