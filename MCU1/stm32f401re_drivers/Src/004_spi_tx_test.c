/*
 * 004_spi_tx_test.c
 *
 *  Created on: Nov 30, 2024
 *      Author: deep
 */

#include "stm32f401re_spi_driver.h"
#include "stm32f401re_gpio_driver.h"
#include <string.h>

/*
 * PB3 -- SPI SCK clock
 * PB4 -- SPI MISO master in slave out
 * PB5 -- SPI MOSI master out slave in
 * PA4 -- SPI NSS slave select
 * Alternate functionality - 5
 * */
void SPI1_GPIOInits(void) {

	/*SPI GPIO configurations*/
	/*common configurations for multiple pins*/
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunc = GPIO_ALTFN_5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	/*SPI SCK*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&SPIPins);

//	/*SPI MISO*/
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
//	GPIO_Init(&SPIPins);

	/*SPI MOSI*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&SPIPins);

//	/*SPI_NSS*/
//	SPIPins.pGPIOx = GPIOA;
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
//	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void) {
	SPI_Handle_t SPIHandle;

	SPIHandle.pSPIx = SPI1;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_BITS8;
	SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPIConfig.SPI_Speed = SPI_SCLK_DIV2; /*spi clock of 8MHz*/
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_EN; /*s/w slave mgmt enabled*/
	SPI_Init(&SPIHandle);
}

int main(void) {

	char user_data[] = "Hello world !\n";

	/*configure GPIO pins to behave as SPI pins*/
	SPI1_GPIOInits();

	/*initialize the spi peripheral*/
	SPI1_Inits();

	/*enable SSI bit to remove MODF error(read about MODF and SSM, SSI bit influence NSS state)*/
	SPI_SSIControl(SPI1, ENABLE);

	/*enable the SPI peripheral*/
	SPI_PeripheralControl(SPI1, ENABLE);

	/*send data*/
	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	/*disable the SPI peripheral*/
	SPI_PeripheralControl(SPI1, DISABLE);

}
