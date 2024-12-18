/*
 * 001_ledtoggle.c
 *
 *  Created on: Nov 25, 2024
 *      Author: deep
 */

#include "stm32f401re_gpio_driver.h"

void delay(void) {
	for(uint32_t i = 0; i <= 500000/2; i++);
}

int main(void) {

	GPIO_Handle_t hgpio;

	hgpio.pGPIOx = GPIOA;
	hgpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	hgpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	hgpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	hgpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_PP;
	hgpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PCLKCtrl(GPIOA, ENABLE);

	GPIO_Init(&hgpio);

	while(1) {
		GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		delay();
	}
}
//end
