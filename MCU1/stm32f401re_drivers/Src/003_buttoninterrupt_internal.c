/*
 * main.c
 *
 *  Created on: Nov 26, 2024
 *      Author: deep
 */

#include "stm32f401re_gpio_driver.h"
#include <string.h>

void delay(void) {
	for(uint32_t i = 0; i <= 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t hgpio; //for led
	GPIO_Handle_t hgpio1; //for button

	/*to make the locak variables default garbage values doesn't corrupt other bits during
	 * configuration*/
	memset(&hgpio, 0 , sizeof(hgpio));
	memset(&hgpio1, 0 , sizeof(hgpio1));

	/*led config*/
	hgpio.pGPIOx = GPIOA;
	hgpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	hgpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	hgpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	hgpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_PP;
	hgpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PCLKCtrl(GPIOA, ENABLE);

	GPIO_Init(&hgpio);

	/*button config*/
	hgpio1.pGPIOx = GPIOC;
	hgpio1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	hgpio1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FALLING_EDGE;
	hgpio1.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PCLKCtrl(GPIOC, ENABLE);

	GPIO_Init(&hgpio1);

	GPIO_IRQPRiorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

}

void EXTI15_10_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
