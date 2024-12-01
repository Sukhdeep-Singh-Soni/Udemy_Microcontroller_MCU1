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
	/*enable spi clock*/
	SPI_ClkCtrl(pSPIHandle->pSPIx, ENABLE);
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

/*
 * @fn		-	SPI_SendData
 * @brief	-	send the data from SPI peripheral(polling mode)
 * @param	-	SPI peripheral base address
 * @param	-	pointer to Tx buffer
 * @param	-	length number of bytes to transmit
 * @ret		-	none
 * @note	-	This is a blocking function
 * */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while(Len > 0) {
		/*wait for TXE bit to set(transmit buffer empty)*/
		while(SPI_GetFlagStatus(pSPIx, SPI_MASK_TXE) == FLAG_RESET);

		/*check the DFF*/
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			/*16-bit data format*/
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len =- 2;
			(uint16_t*)pTxBuffer++;
		} else {
			/*8 bit frame format*/
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 * @fn		-	SPI_DeIint
 * @brief	-	reset the SPIx peripheral
 * @param	-	SPI peripheral base address
 * @ret		-	none
 * @note	-	none
 * */
void SPI_ReadData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

}

/*
 * @fn		-	SPI_GetFlagStatus
 * @brief	-	give the status of the specified flag bit in the SR register
 * @param	-	SPI peripheral base address
 * @param	-	SPI flag mask like for eg. TXE bit (1 << 1)
 * @ret		-	1 if bit is set or 0 otherwise
 * @note	-	Helping function
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName) {
	if(pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); /*enable the peripheral*/
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); /*disable the peripheral*/
	}
}

void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI); /*enable the peripheral*/
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); /*disable the peripheral*/
	}
}














