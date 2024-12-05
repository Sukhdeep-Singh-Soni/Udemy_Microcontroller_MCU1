/*
 * stm32f401re_spi_driver.c
 *
 *  Created on: Nov 29, 2024
 *      Author: deep
 */

#include "stm32f401re_spi_driver.h"

/*
 * private function to this file(helper functions)
 * */
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
 * @fn		-	SPI_ReadData
 * @brief	-	receive data from SPI peripheral
 * @param	-	SPI peripheral base address
 * @param	-	address of buffer to read data into
 * @param	-	length amount of data to read
 * @ret		-	none
 * @note	-	none
 * */
void SPI_ReadData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while(Len > 0) {
		/*wait for RXNE bit to set(transmit buffer empty)*/
		while(SPI_GetFlagStatus(pSPIx, SPI_MASK_RXNE) == FLAG_RESET);

		/*check the DFF*/
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			/*16-bit data format, get data from DR into RXBuffer*/
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len =- 2;
			(uint16_t*)pRxBuffer++;
		} else {
			/*8 bit frame format*/
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
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

/*
 * @fn		-	SPI_PeripheralControl
 * @brief	-	enable or disable SPIx peripheral
 * @param	-	SPI peripheral base address
 * @param	-	whether to enable or disable the peripheral
 * @ret		-	none
 * @note	-	none
 * */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); /*enable the peripheral*/
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); /*disable the peripheral*/
	}
}

/*
 * @fn		-	SPI_SSIControl
 * @brief	-	enable or disable the spi SSI bit if SSM is selected
 * @param	-	SPI peripheral base address
 * @param	-	whether to enable or disable SSI bit
 * @ret		-	none
 * @note	-	none
 * */
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 * @fn		-	SPI_IRQInterruptConfig
 * @brief	-	enable or disable SPI interrupt delivery at NVIC side
 * @param	-	IRQ number for the SPI interrupt
 * @param	-	to enable or disable the interrupt
 * @ret		-	none
 * @note	-	NVIC_ISER0 and NVIC_ICER0 are base addresses of interrupt set enable register and interrupt clear enable register respectively
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	uint8_t iserx = IRQNumber / 32;
	uint8_t iserx_section = IRQNumber % 32;
	if(EnorDi == ENABLE) {
		*(NVIC_ISER0 + iserx) |= (1 << iserx_section); /*set interrupt enable bit in NVIC*/
	} else {
		*(NVIC_ICER0 + iserx) &= ~(1 << iserx_section); /*clear interrupt enable bit*/
	}
}

/*
 * @fn		-	SPI_IRQPRiorityConfig
 * @brief	-	configures SPI interrupt prioriy in one of the NVIC priority registers
 * @param	-	IRQ number for the SPI peripheral connected to NVIC
 * @param	-	IRQ priority to configure for the interrupt
 * @ret		-	none
 * @note	-	none
 * */
void SPI_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amt = (iprx_section * 8) + (8 - NVIC_PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amt);
}

/*
 * @fn		-	SPI_SendDataIT
 * @brief	-	send SPI data in interrupt mode
 * @param	-	pointer to SPI handle structure
 * @param	-	pointer to transmit buffer
 * @param	-	length amount to transmit
 * @ret		-	status of the SPI peripheral
 * @note	-	This is a non-blocking function
 * */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX) {
		/*1.Save the Tx buffer address and length info in some globel variables*/
		pSPIHandle->TxLen = Len;
		pSPIHandle->pTxBuffer = pTxBuffer;

		/*2. Change state of SPI peripheral to busy in Tx
		 * so that no other code can take over SPI peripheral until transmission is over*/
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/*3. Enable the SPI TXEIE control bit to get interrupt when TXE bit is set in status register*/
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		/*4. The DR register value update will happen in SPI interrupt handler */
	}
	return state;
}

/*
 * @fn		-	SPI_ReadDataIT
 * @brief	-	receive data from SPI peripheral in interrupt mode
 * @param	-	pointer to SPI handle structure
 * @param	-	pointer to reveice buffer
 * @param	-	length amount to read
 * @ret		-	status of SPI reception
 * @note	-	This is a non-blocking function
 * */
uint8_t SPI_ReadDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX) {
		/*1.Save the Rx buffer address and length info in some globel variables*/
		pSPIHandle->RxLen = Len;
		pSPIHandle->pRxBuffer = pRxBuffer;

		/*2. Change state of SPI peripheral to busy in Rx
		 * so that no other code can take over SPI peripheral until reception is over*/
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/*3. Enable the SPI RXNEIE control bit to get interrupt when RXNE bit is set in status register*/
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		/*4. The DR register value update will happen in SPI interrupt handler */
	}
	return state;
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint32_t temp1, temp2;

	/*check if interrupt come from transmit buffer empty, TXE event*/
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2) {
		//handle transmission
		spi_tx_interrupt_handle(pSPIHandle);
	}

	/*check if interrupt come from recieve buffer not empty, RXNE event*/
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2) {
		//handle transmission
		spi_rx_interrupt_handle(pSPIHandle);
	}

	/*check if interrupt come from an error , OVR or CRC etc event*/
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR); /*checking for Overrun error*/
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2) {
		//handle transmission
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/*put data(buffer pointer saved during SPI_SendDataIT) into data register amd check the DFF*/
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		/*16-bit data format*/
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen =- 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		/*8 bit frame format*/
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen) {
		/*TxLen is zero, so close the transmission and inform the application that
		 * Tx is over*/

		/*This prevents interrupts, from setting up of TXE flag*/
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/*check the DFF*/
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		/*16-bit data format, get data from DR into RXBuffer*/
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen =- 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else {
		/*8 bit frame format*/
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen) {
		/*TxLen is zero, so close the transmission and inform the application that
		 * Tx is over*/

		/*This prevents interrupts, from setting up of TXE flag*/
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	uint16_t tmp;
	/*clear the ovr flag if SPI in not in Tx state*/
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX ) {
		tmp = pSPIHandle->pSPIx->DR;
		tmp = pSPIHandle->pSPIx->SR;
	}
	(void)tmp;
	/*inform the application about the overrun error and let the application
	 * handle / clear the overrun flag if spi is in transmission state*/
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t SPIEvent) {
		/*weak implementation of the callback function
		 * if application doesn't inplement this function
		 * then this function will be called otherwise the application
		 * implementation will be called*/
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
void SPI_ClearOVRError(SPI_RegDef_t *pSPIx) {
	uint16_t tmp;
	tmp = pSPIx->DR;
	tmp = pSPIx->SR;
	(void)tmp; /*to remove the warning of unused variable*/
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); /*disable TXE event interrupt*/
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*
 * @fn		-	SPI_IRQHandling
 * @brief	-	handles the SPI interrupt when ISR comes
 * @param	-	pointer to SPI handle structure
 * @ret		-	none
 * @note	-	none
 * */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); /*disable TXE event interrupt*/
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}






