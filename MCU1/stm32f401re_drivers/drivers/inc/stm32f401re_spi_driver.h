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
	uint8_t SPI_DeviceMode;	/*device mode (master or slave) goto @SPI_DeviceMode*/
	uint8_t SPI_BusConfig; /*full-duplex/half-duplex/simplex, goto @SPI_BusConfig*/
	uint8_t SPI_Speed; /* goto @SPI_Speed*/
	uint8_t SPI_DFF; /*data frame format(8-bit/16-bit), goto @SPI_DFF*/
	uint8_t SPI_CPOL; /*Clock polarity, goto @SPI_CPOL*/
	uint8_t SPI_CPHA; /*CLock Phase, goto @SPI_CPHA*/
	uint8_t SPI_SSM; /*Slave select management(s/w or h/w), goto @SPI_SSM*/
}SPI_Config_t;

/*
 * SPI Handle structure
 * */
typedef struct {
	SPI_RegDef_t *pSPIx; /*pointer SPI peripheral base address*/
	SPI_Config_t SPIConfig; /*SPI peripheral configuration settings*/
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * */
#define SPI_MODE_MASTER		0x01 /*master configuration*/
#define SPI_MODE_SLAVE		0x00 /*slave configuration*/

/*
 * @SPI_BusConfig
 * */
#define SPI_BUS_CONFIG_FD				0 /*full duplex*/
#define SPI_BUS_CONFIG_HD				1 /*half duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_RDONLY	3 /*simplex with read only mode(simplex with tx only is same as full duplex with recieve line{MISO} not connected)*/

/*
 * @SPI_Speed
 * */
#define SPI_SCLK_DIV2		0 /*spi serial clock = pclk / 2*/
#define SPI_SCLK_DIV4		1 /*spi serial clock = pclk / 4*/
#define SPI_SCLK_DIV8		2 /*spi serial clock = pclk / 8*/
#define SPI_SCLK_DIV16		3 /*spi serial clock = pclk / 16*/
#define SPI_SCLK_DIV32		4 /*spi serial clock = pclk / 32*/
#define SPI_SCLK_DIV64		5 /*spi serial clock = pclk / 64*/
#define SPI_SCLK_DIV128		6 /*spi serial clock = pclk / 128*/
#define SPI_SCLK_DIV256		7 /*spi serial clock = pclk / 256*/

/*
 * @SPI_DFF
 * */
#define SPI_DFF_BITS8		0 /*data frame format 8-bits*/
#define SPI_DFF_BITS16		1 /*data frame format 16-bits*/

/*
 * @SPI_CPOL
 * */
#define SPI_CPOL_HIGH		1 /*clock polarity high at idle*/
#define SPI_CPOL_LOW		0 /*clock polarity low at idle state*/

/*
 * @SPI_CPHA
 * */
#define SPI_CPHA_HIGH		1 /*data sample/cpature at 2nd edge of clock and data send at 1st edge*/
#define SPI_CPHA_LOW		0 /*data sample at 1st edge of serial clock and send at 2nd egde*/

/*
 * @SPI_SSM
 * */
#define SPI_SSM_EN	1 /*software slave management enabled*/
#define SPI_SSM_DI	0 /*hardware slave mgmt enables(SSM disabled)*/

/*
 * SPI CR1 register masking bit definitions
 * */
#define SPI_MASK_TXE	(1 << SPI_SR_TXE)
#define SPI_MASK_RXNE	(1 << SPI_SR_RXNE)
#define SPI_MASK_BSY	(1 << SPI_SR_BSY)

/*
 * ##################### API's provided by this driver ###############################
 * */

/*
 * SPI clock enable or disable macros
 * */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

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

/*
 * Some other miscellaneous SPI api's
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);


#endif /* INC_STM32F401RE_SPI_DRIVER_H_ */
