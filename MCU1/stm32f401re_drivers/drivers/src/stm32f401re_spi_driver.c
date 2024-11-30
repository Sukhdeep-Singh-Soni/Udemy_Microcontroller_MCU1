/*
 * stm32f401re_spi_driver.c
 *
 *  Created on: Nov 29, 2024
 *      Author: deep
 */

#include "stm32f401re_spi_driver.h"

/*
 * @fn		-	SPI_ClkCtrl
 * @brief	-	enable or disable SPIx peripheral clock
 * @param	-	SPI peripheral base address
 * @ret		-	none
 * @note	-	none
 * */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_CLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_CLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_CLK_EN();
		} else if(pSPIx == SPI4) {
			SPI4_CLK_EN();
		}
	} else {
		if(pSPIx == SPI1) {
			SPI1_CLK_DI();
		} else if(pSPIx == SPI2) {
			SPI2_CLK_DI();
		} else if(pSPIx == SPI3) {
			SPI3_CLK_DI();
		} else if(pSPIx == SPI4) {
			SPI4_CLK_DI();
		}
	}
}

/*
 * @fn		-	SPI_Init
 * @brief	-	initialize the SPI peripheral
 * @param	-	SPI peripheral handle structure pointer
 * @ret		-	none
 * @note	-	none
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	/*configure SPI CR1 register*/
	uint32_t tempreg = 0;

	/*1. configure the DeviceMode*/
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_MODE_MASTER) {
		tempreg |= (1 << SPI_CR1_MSTR); /*set*/
	} else {
		tempreg &= ~(1 << SPI_CR1_MSTR); /*clear*/
	}
	/*2. configure the BusConfig*/
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		/*clear the bidirectional*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		/*set the bidirectional mode*/
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RDONLY) {
		/*clear the bidirectional mode and set the RXONLY bit for force receive and produce clock even on master receive*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
	/*3. configure the SPI_Speed*/
	tempreg |= ((pSPIHandle->SPIConfig.SPI_Speed & 0x07) << SPI_CR1_BR);
	/*4. configure the SPI_DFF*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	/*5. configure the SPI_CPOL*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	/*6. configure the SPI_CPHA*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	/*7. configure the SPI_SSM*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	/*put all configuration into SPI Cr1 register*/
	pSPIHandle->pSPIx->CR1 = tempreg; /*this time we can do direct initialization as register do not have any reserved bits and we initialize tempreg with 0 first*/
}

/*
 * @fn		-	SPI_DeIint
 * @brief	-	reset the SPIx peripheral
 * @param	-	SPI peripheral base address
 * @ret		-	none
 * @note	-	none
 * */
void SPI_DeIint(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1) {
		SPI1_RESET_REG();
	} else if(pSPIx == SPI2) {
		SPI2_RESET_REG();
	} else if(pSPIx == SPI3) {
		SPI3_RESET_REG();
	} else if(pSPIx == SPI4) {
		SPI4_RESET_REG();
	}
}
