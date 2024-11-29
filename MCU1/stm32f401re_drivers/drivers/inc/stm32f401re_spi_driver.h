/*
 * stm32f401re_spi_driver.h
 *
 *  Created on: Nov 29, 2024
 *      Author: deep
 */

#ifndef INC_STM32F401RE_SPI_DRIVER_H_
#define INC_STM32F401RE_SPI_DRIVER_H_

#include "stm32f401re.h"

/*
 * SPI peripheral configuration structure
 * */
typedef struct {
	uint8_t SPI_DeviceMode;	/*device mode (master or slave)*/
	uint8_t SPI_BusConfig; /*full-duplex/half-duplex/simplex*/
	uint8_t SPI_Speed; /**/
	uint8_t SPI_DFF; /*data frame format(8-bit/16-bit)*/
	uint8_t SPI_CPOL; /*Clock polarity*/
	uint8_t SPI_CPHA; /*CLock Phase*/
	uint8_t SPI_SSM; /*Slave select management(s/w or h/w)*/
}SPI_Config_t;

/*
 * SPI Handle structure
 * */
typedef struct {
	SPI_RegDef_t *pSPIx; /*pointer SPI peripheral base address*/
	SPI_Config_t SPIConfig; /*SPI peripheral configuration settings*/
}SPI_Handle_t;

/*
 * ##################### API's provided by this driver ###############################
 * */

/*
 * SPI clock enable or disable macros
 * */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx);

/*
 * SPI Init and DeInit
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeIint(SPI_RegDef_t *pSPIx);

/*
 * SPI Tx and Rx
 * */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReadData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Handling and Interrupt config
 * */
void GPIO_IRQHandling(SPI_Handle_t *pSPIHandle);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPRiorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


#endif /* INC_STM32F401RE_SPI_DRIVER_H_ */
