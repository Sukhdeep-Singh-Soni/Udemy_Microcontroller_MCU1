/*
 * stm32f401re_i2c_driver.c
 *
 *  Created on: Dec 7, 2024
 *      Author: deep
 */


#include "stm32f401re_i2c_driver.h"

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

