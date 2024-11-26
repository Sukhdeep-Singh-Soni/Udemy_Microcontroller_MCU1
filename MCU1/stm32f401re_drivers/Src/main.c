/*
 * main.c
 *
 *  Created on: Nov 26, 2024
 *      Author: deep
 */

#include "stm32f401re_gpio_driver.h"

int main(void) {

}

void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(0);
}
