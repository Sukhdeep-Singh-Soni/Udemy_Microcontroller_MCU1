/*
 * stm32f401re_i2c_driver.h
 *
 *  Created on: Dec 7, 2024
 *      Author: deep
 */

#ifndef INC_STM32F401RE_I2C_DRIVER_H_
#define INC_STM32F401RE_I2C_DRIVER_H_

#include "stm32f401re.h"

/*
 * Configuration structure for I2Cx peripheral
 * */
typedef struct {
	uint32_t I2C_SCLSpeed;		/*i2c serial clock speed like 100Khz or 400Khz etc, goto @I2C_SCLSpeed*/
	uint8_t  I2C_DeviceAddress; /*i2c slave/device address, configured by user*/
	uint8_t  I2C_ACKControl;	/*whether to enable or disable acking, goto @I2C_ACKControl*/
	uint16_t I2C_FMDutyCycle;	/*i2c fast mode duty cycle, goto @I2C_FMDutyCycle*/
}I2C_Config_t;

/*
 * @I2C_SCLSpeed
 * */
#define I2C_SPEED_SM	100000U /*standard mode 100KHz*/
#define I2C_SPEED_FM4K	400000U /*fast mode 400KHz*/
#define I2C_SPEED_FM2K	200000U /*fast mode 200KHz*/

/*
 * @I2C_ACKControl
 * */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 * */
#define I2C_DUTY_2			0
#define I2C_DUTY_16_9		1

/*
 * Handle structure for I2Cx peripheral
 * */
typedef struct {
	I2C_RegDef_t *pI2Cx; /*pointer to I2C peripheral*/
	I2C_Config_t I2C_Config; /*configuration structure for th i2c peripheral*/
	uint8_t 	 *pTxBuffer; /*pointer to application Tx buffer*/
	uint8_t		 *pRxBuffer; /*pointer to app. Rx buffer*/
	uint32_t	 TxLen; /*length of transmit buffer*/
	uint32_t	 RxLen; /*length of receive buffer*/
	uint8_t 	 TxRxState; /*state of the I2C peripheral, goto @I2C_States*/
	uint8_t		 DevAddr; /*I2C device address holder*/
	uint32_t 	 RxSize; /*app. Rx size*/
	uint8_t		 Sr; /* used in repeated start senarios*/
}I2C_Handle_t;

/*
 * @I2C_States
 * I2C application states
 * */
#define I2C_STATE_READY			0
#define I2C_STATE_BUSY_IN_TX	1
#define I2C_STATE_BUSY_IN_RX	2

/*
 * I2C SR1 and SR2 register masking bit definitions
 * */
#define I2C_MASK_SB			(1 << I2C_SR1_SB)
#define I2C_MASK_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_MASK_BTF		(1 << I2C_SR1_BTF)
#define I2C_MASK_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_MASK_RxNE		(1 << I2C_SR1_RxNE)
#define I2C_MASK_TxE		(1 << I2C_SR1_TxE)
#define I2C_MASK_BERR		(1 << I2C_SR1_BERR)
#define I2C_MASK_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_MASK_AF			(1 << I2C_SR1_AF)
#define I2C_MASK_OVR		(1 << I2C_SR1_OVR)
#define I2C_MASK_TIMEOUT	(1 << I2C_SR1_TIMEOUT)
#define I2C_MASK_MSL		(1 << I2C_SR1_MSL)
#define I2C_MASK_BUSY		(1 << I2C_SR1_BUSY)
#define I2C_MASK_TRA		(1 << I2C_SR1_TRA
#define I2C_MASK_DUALF		(1 << I2C_SR1_DUALF)


/*
 * I2C repeated start macros
 * */
#define I2C_ENABLE_SR		RESET
#define I2C_DISABLE_SR		SET

/*
 * ##################### API's provided by this driver ###############################
 * */

/*
 * I2C clock enable or disable macros
 * */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C Init and DeInit
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeIint(I2C_RegDef_t *pI2Cx);

/*
 * I2C Tx and Rx
 * */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
/*
 * IRQ Handling and Interrupt config
 * */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Some other miscellaneous I2C api's
 * */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint8_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2CEvent); /*I2CEvent from @I2CEvent*/
void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx);
void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F401RE_I2C_DRIVER_H_ */
