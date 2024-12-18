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
	uint8_t GPIO_PinNumber; /*possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;	/*possible values from @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;	/*possible values from @GPIO_PIN_OSPEED*/
	uint8_t GPIO_PinPuPd;	/*possible values from @GPIO_PIN_PUPDR*/
	uint8_t GPIO_PinOPType;	/*possible values from @GPIO_PIN_OPTYPE*/
	uint8_t GPIO_PinAltFunc;/*possible values from @GPIO_PIN_ALT_FUN*/
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; /*This contains the base addr of GPIO peripheral to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig; /*contains the gpio pin configuration*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODE
 * GPIO possible modes
 * */
#define GPIO_MODE_IN						0
#define GPIO_MODE_OUT						1
#define GPIO_MODE_ALTFN						2
#define GPIO_MODE_ANALOG					3
#define GPIO_MODE_IT_FALLING_EDGE			4
#define GPIO_MODE_IT_RISING_EDGE			5
#define GPIO_MODE_IT_RISING_FALLING_EDGE	6

/*
 * @GPIO_PIN_OPTYPE
 * possible GPIO output types
 * */
#define GPIO_OP_PP	0	/*push pull*/
#define GPIO_OP_OD	1	/*open drain*/

/*
 * @GPIO_PIN_OSPEED
 * possible GPIO output speed
 * */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3

/*
 * @GPIO_PIN_PUPDR
 * GPIO possible values for pull up pull down
 */
#define GPIO_NO_PUPD		0 /*no pullup pulldown*/
#define GPIO_PULL_UP		1
#define GPIO_PULL_DOWN		2

/*
 * @GPIO_PIN_ALT_FUN
 * GPIO possible alternate function value
 * */
#define GPIO_ALTFN_0		0
#define GPIO_ALTFN_1		1
#define GPIO_ALTFN_2		2
#define GPIO_ALTFN_3		3
#define GPIO_ALTFN_4		4
#define GPIO_ALTFN_5		5
#define GPIO_ALTFN_6		6
#define GPIO_ALTFN_7		7
#define GPIO_ALTFN_8		8
#define GPIO_ALTFN_9		9
#define GPIO_ALTFN_10		10
#define GPIO_ALTFN_11		11
#define GPIO_ALTFN_12		12
#define GPIO_ALTFN_13		13
#define GPIO_ALTFN_14		14
#define GPIO_ALTFN_15		15

/*
 * @GPIO_PIN_NUMBER
 * possible GPIO pin values
 * */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15


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
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * Interrupt configuration and Handling
 * */
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F401RE_GPIO_DRIVER_H_ */
