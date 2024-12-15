/*
 * stm32f401re_i2c_driver.c
 *
 *  Created on: Dec 7, 2024
 *      Author: deep
 */


#include "stm32f401re_i2c_driver.h"

uint16_t AHB1_Prescalar[] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Presaclar[] = {2, 4, 8, 16};

uint32_t RCC_GetPclk1Freq(void);
uint32_t RCC_GetPLLClock(void);

/*
 * @fn		-	I2C_ClkCtrl
 * @brief	-	enable or disable I2Cx peripheral clock
 * @param	-	I2C peripheral base address
 * @param	-	whether to enable to disable the clock for I2Cx peripheral
 * @ret		-	none
 * @note	-	none
 * */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pI2Cx == I2C1) {
			I2C1_CLK_EN();
		} else if(pI2Cx == I2C2) {
			I2C2_CLK_EN();
		} else if(pI2Cx == I2C3) {
			I2C3_CLK_EN();
		}
	} else {
		if(pI2Cx == I2C1) {
			I2C1_CLK_DI();
		} else if(pI2Cx == I2C2) {
			I2C2_CLK_DI();
		} else if(pI2Cx == I2C3) {
			I2C3_CLK_DI();
		}
	}
}

/*
 * @fn		-	I2C_DeIint
 * @brief	-	reset the I2Cx peripheral
 * @param	-	I2C peripheral base address
 * @ret		-	none
 * @note	-	none
 * */
void I2C_DeIint(I2C_RegDef_t *pI2Cx) {
	if(pI2Cx == I2C1) {
		I2C1_RESET_REG();
	} else if(pI2Cx == I2C2) {
		I2C2_RESET_REG();
	} else if(pI2Cx == I2C3) {
		I2C3_RESET_REG();
	}
}

/*
 * @fn		-	I2C_PeripheralControl
 * @brief	-	enable or disable I2Cx peripheral
 * @param	-	I2C peripheral base address
 * @param	-	whether to enable or disable the peripheral
 * @ret		-	none
 * @note	-	none
 * */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * @fn		-	I2C_IRQInterruptConfig
 * @brief	-	configure I2C interrupt in NVIC registers
 * @param	-	I2Cx peripheral interrupt number
 * @param	-	whether to enable or disable I2Cx peripheral interrupt
 * @ret		-	none
 * @note	-	none
 * */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	uint8_t iserx = IRQNumber / 32;
	uint8_t iserx_section = IRQNumber % 32;
	if(EnorDi == ENABLE) {
		*(NVIC_ISER0 + iserx) |= (1 << iserx_section); /*set interrupt enable bit in NVIC*/
	} else {
		*(NVIC_ICER0 + iserx) &= ~(1 << iserx_section); /*clear interrupt enable bit*/
	}
}

/*
 * @fn		-	I2C_IRQPRiorityConfig
 * @brief	-	configures I2Cx peripheral priority
 * @param	-	I2Cx IRQnumber to configure priority for
 * @param	-	I2Cx priority to configure for the corresponding I2Cx peripheral interrupt
 * @ret		-	none
 * @note	-	none
 * */
void I2C_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amt = (iprx_section * 8) + (8 - NVIC_PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amt);
}

/*
 * @fn		-	I2C_Init
 * @brief	-	Initialize the I2C peripheral
 * @param	-	I2Cx peripheral handle structure pointer
 * @ret		-	none
 * @note	-	none
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp = 0;
	/*configure the ACKing in I2C*/
	temp |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= temp;

	/*configure the i2c scl frequency*/
	temp = 0;
	temp = ((RCC_GetPclk1Freq() / 1000000) << I2C_CR2_FREQ);
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x3f);

	/*configure the device own address*/
	temp = 0;
	temp |= (1 << 14); /*according to reference manual requirement*/
	temp &= ~(1 << 15); /*clear the addr mode for 7-bit addressing*/
	temp |= ((pI2CHandle->I2C_Config.I2C_DeviceAddress) << 1); /*7-bit address starts from bit 1*/
	pI2CHandle->pI2Cx->OAR1 |= temp;

	/*configure the clock control speed (CCR calculations)*/
	temp = 0;
	uint16_t ccr_value = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM) {
		/*standard mode*/
		temp &= ~(1 << I2C_CCR_F_S); /*set standard mode*/
		ccr_value |= (RCC_GetPclk1Freq() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		temp |= (ccr_value & 0xfff);
	} else {
		/*fast mode*/
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_DUTY_2) {
				temp |= (1 << I2C_CCR_F_S); /*set fast mode*/
				ccr_value = (RCC_GetPclk1Freq() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_DUTY_16_9) {
			temp |= (1 << I2C_CCR_F_S); /*set fast mode*/
			ccr_value |= (RCC_GetPclk1Freq() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		temp |= (ccr_value & 0xfff);
	}
	pI2CHandle->pI2Cx->CCR |= temp;

	/*TRISE calculations*/
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM) {
		/*standard mode*/
		temp = (RCC_GetPclk1Freq() / 1000000U) + 1;
	} else {
		/*fast mode*/
		temp = ((RCC_GetPclk1Freq() * 300U) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE |= (temp & 0x3f);
}


/*
 * @fn		-	RCC_GetPclkFreq
 * @brief	-	get the system clock frequency
 * @param	-	none
 * @ret		-	none
 * @note	-	none
 * */
uint32_t RCC_GetPclk1Freq(void) {
	uint32_t cfgr, ahb1p, apb1p;
	uint32_t SystemClk, pclk;

	cfgr = ((RCC->CFGR >> 2) & 0x3);

	if(cfgr == 0) {
		/*HSI is used as system clock*/
		SystemClk = 16000000;
	} else if(cfgr == 1) {
		/*HSE is used as system clock*/
		/*in nucleo-f401re no external crystal is used*/
		SystemClk = 0;
	} else if(cfgr == 2) {
		/*PLL clock calculate*/
		SystemClk = RCC_GetPLLClock();
	}

	cfgr = ((RCC->CFGR >> 4) & 0xf);

	if(cfgr < 8) {
		ahb1p = 1;
	} else {
		ahb1p = AHB1_Prescalar[cfgr - 8];
	}

	cfgr = ((RCC->CFGR >> 10) & 0x7);

	if(cfgr < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_Presaclar[cfgr - 4];
	}

	pclk = ((SystemClk / ahb1p) / apb1p);

	return pclk;
}

uint32_t RCC_GetPLLClock(void) {
	/*not implemented yet*/
	return 0;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr) {
	/*Generate the start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/*confirm that start generation is completed by checking the SB flag in the SR1
	 * Note: Until SB is cleared SCL is stretched to low*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_SB));

	/*send the address of the slave with r/w bit set to w(0) (total 8 bits)*/
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	/*confirm the address phase is completed by checking the ADDr flag in SR register*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_ADDR));

	/*clear the ADDR flag according to its software sequence
	 * Note: Until ADDR is cleared SCL is stretched to low*/
	I2C_ClearADDRFlag(pI2CHandle);

	/*send data until length becomes 0*/
	while(len > 0) {
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_TxE)); /*wait until TXE is set*/
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
	}

	/*When len becomes zero, wait for TXE=1 and BTF=1 before generating the STOP condition
	 * Note: TXE=1 and BTF=1 , means that both SR and DR are empty and next transmission
	 * should begin when BTF=1, SCL will be stretched(pulled to low)*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_TxE));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_BTF));

	/*Generate STOP condition and master need not wait for the completion of stop condition
	 * Note: generating STOP, automatically clears the BTF*/
	if(Sr == I2C_DISABLE_SR) {
		I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr) {
	//Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//wait for SB bit to become zero
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_SB));

	//send the address of the slave with r/w bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//wait for address phase to get completed
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_ADDR));

	//procedure to read only 1 byte from slave
	if(len == 1) {
		//set ACK to 0
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until rxne becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_RxNE));

		//set STOP bit to 1
		if(Sr == I2C_DISABLE_SR) {
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
		}

		//read data into the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data when len > 1
	if(len > 1) {
		for(int i = len; i > 0; i--) {
			//clear the addr flag
			I2C_ClearADDRFlag(pI2CHandle);

			if(i == 2) {
				//disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate stop condition
				if(Sr == I2C_DISABLE_SR) {
					I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
				}
			}

			//wait for rxne flag to set
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_RxNE));

			//read data from data register
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
		}
	}
	//re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*
 * @fn		-	I2C_GetFlagStatus
 * @brief	-	give the status of the specified flag bit in the SR register
 * @param	-	I2C peripheral base address
 * @param	-	I2C flag mask like for eg. TXE bit (1 << 1)
 * @ret		-	1 if bit is set or 0 otherwise
 * @note	-	Helping function
 * */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint8_t FlagName) {
	if(pSPIx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1; /*create espace for read/write bit*/
	SlaveAddr &= ~(0x01); /*clear for write*/
	pI2Cx->DR = SlaveAddr;
}

void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1; /*create espace for read/write bit*/
	SlaveAddr |= 0x01; /*set to read*/
	pI2Cx->DR = SlaveAddr;
}

void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint16_t temp;

	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		//device in master mode
		if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				//disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//clear the ADDr flag
				temp = pI2CHandle->pI2Cx->SR1;
				temp = pI2CHandle->pI2Cx->SR2;
				(void)temp;
			}
		} else {
			//clear the ADDr flag
			temp = pI2CHandle->pI2Cx->SR1;
			temp = pI2CHandle->pI2Cx->SR2;
			(void)temp;
		}
	} else {
		//device in slave mode
		//clear the ADDr flag
		temp = pI2CHandle->pI2Cx->SR1;
		temp = pI2CHandle->pI2Cx->SR2;
		(void)temp;
	}
}

void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if(EnorDi == I2C_ACK_ENABLE) {
		//enable acking
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		//disable acking
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_STATE_BUSY_IN_TX) && (busystate != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_STATE_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3) {
		//The interrupt is generated due to SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3) {
		//interrupt is generated due to ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3) {
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX) {
			//check if Txe is set
			if(pI2CHandle->pI2Cx->SR1 & I2C_SR1_TxE) {
				//BTF=1, TxE=1

				if(pI2CHandle->TxLen == 0) {
					//generate stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

					//reset all the member elements of handle structure
					I2C_CloseSendData(pI2CHandle);

					//notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3) {
		//STOPF flag is set
		//clear the STOPF flag (i.e. read SR1 followed by write to CR1)
//		temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF); read already done above
		pI2CHandle->pI2Cx->CR1 |= 0x0000; /*does nothing just write operation is performed without changing the previous bits*/

		//notify the application that a STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3) {
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//TxE flag is set
			//we have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_TX) {
				I2C_MasterHandleTxEInterrupt(pI2CHandle);
			}
		} else {
			//slave
			//make sure the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3) {
		//RxNE flag is set
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//master mode
			//we have to start data reception
			if(pI2CHandle->TxRxState == I2C_STATE_BUSY_IN_RX) {
				I2C_MasterHandleRxNEInterrupt(pI2CHandle);
			}
		} else {
			//slave
			//make sure the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RECV);
			}
		}
	}
}

void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->RxSize == 1) {
		//read one byte
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//decrement length
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1) {

		if(pI2CHandle->RxLen == 2) {
			//disable acking
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		//read one byte
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//decrement length
		pI2CHandle->RxLen--;

		//increment receive buffer pointer
		pI2CHandle->pRxBuffer++;
	}

	if(pI2CHandle->RxSize == 0) {
		//close the i2c data reception and notyfy the application

		//generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

		//close the i2c rx
		I2C_CloseReceiveData(pI2CHandle);

		//notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_MasterHandleTxEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->TxLen > 0) {
		//put one byte into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//decrement the length
		pI2CHandle->TxLen--;

		//increment the data buffer address
		pI2CHandle->pTxBuffer++;
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	//disable the interrupt for i2c events and txe and rxne events
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//reset the i2c handle structure
	pI2CHandle->TxRxState = I2C_STATE_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	//disable the interrupt for i2c events and txe and rxne events
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//reset the i2c handle structure
	pI2CHandle->TxRxState = I2C_STATE_READY;
	pI2CHandle->RxLen = 0;
	pI2CHandle->pRxBuffer = NULL;

	//enable the acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);

	}

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
	return (uint8_t)pI2Cx->DR;
}











