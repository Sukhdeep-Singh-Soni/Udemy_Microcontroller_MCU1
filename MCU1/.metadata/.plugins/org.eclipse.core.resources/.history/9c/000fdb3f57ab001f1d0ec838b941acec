/*
 * 002_ledbutton.c
 *
 *  Created on: Nov 25, 2024
 *      Author: deep
 */

#include "stm32f401re_gpio_driver.h"

#define LOW				0
#define BUTTON_PRESSED	LOW

void delay(void) {
	for(uint32_t i = 0; i <= 500000/2; i++);
}

int main(void) {

	GPIO_Handle_t hgpio; //for led
	GPIO_Handle_t hgpio1; //for button

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
	hgpio1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	hgpio1.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PCLKCtrl(GPIOC, ENABLE);

	GPIO_Init(&hgpio1);

	while(1) {

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BUTTON_PRESSED) {
			delay(); /*for button debouncing*/
			GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	}
}
//end
