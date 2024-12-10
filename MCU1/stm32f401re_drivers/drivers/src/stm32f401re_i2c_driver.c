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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr) {
	/*Generate the start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/*confirm that start generation is completed by checking the SB flag in the SR1
	 * Note: Until SB is cleared SCL is stretched to low*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_SB));

	/*send the address of the slave with r/w bit set to w(0) (total 8 bits)*/
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	/*confirm the address phase is completed by checking the ADDr flag in SR register*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_MASK_ADDR));

	/*clear the ADDR flag according t its software sequence
	 * Note: Until ADDR is cleared SCL is stretched to low*/
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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
	I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
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

void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1; /*create espace for read/write bit*/
	SlaveAddr &= ~(0x01); /*clear for write*/
	pI2Cx->DR = SlaveAddr;
}

void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint16_t temp = pI2Cx->SR1;
	temp = pI2Cx->SR2;
	(void)temp;
}

void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

