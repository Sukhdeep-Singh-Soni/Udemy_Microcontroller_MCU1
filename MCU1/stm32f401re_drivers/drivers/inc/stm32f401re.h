/*
 * stm32f401re.h
 *
 *  Created on: Nov 24, 2024
 *      Author: deep
 */

#ifndef INC_STM32F401RE_H_
#define INC_STM32F401RE_H_

#include <stdint.h>

#define __vo volatile

/*
 * ########################## Precessor Specific definitions ##############################
 * */

/*
 * Arm Cortex M4 NVIC ISER register definitions(Interrupt set-enable registers)
 * */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10CU)

/*
 * Arm Cortex M4 NVIC ICER register definitions(Interrupt clear-enable registers)
 * */
#define NVIC_ICER0		((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1		((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3		((__vo uint32_t*)0XE000E18CU)

/*
 * Arm Cortex M4 NVIC IPR register address definitions(Interrupt Priority registers)
 * */
#define NVIC_IPR_BASEADDR	((__vo uint32_t*)0xE000E400U)
/*
 * Arm Cortec M4 priority bits implemented are 4
 * */
#define NVIC_PRIORITY_BITS_IMPLEMENTED		4

/*
 * NVIC priority values macros
 * */
#define NVIC_IRQ_PRIORITY0	0
#define NVIC_IRQ_PRIORITY1	1
#define NVIC_IRQ_PRIORITY2	2
#define NVIC_IRQ_PRIORITY3	3
#define NVIC_IRQ_PRIORITY4	4
#define NVIC_IRQ_PRIORITY5	5
#define NVIC_IRQ_PRIORITY6	6
#define NVIC_IRQ_PRIORITY7	7
#define NVIC_IRQ_PRIORITY8	8
#define NVIC_IRQ_PRIORITY9	9
#define NVIC_IRQ_PRIORITY10	10
#define NVIC_IRQ_PRIORITY11	11
#define NVIC_IRQ_PRIORITY12	12
#define NVIC_IRQ_PRIORITY13	13
#define NVIC_IRQ_PRIORITY14	14
#define NVIC_IRQ_PRIORITY15	15

/*
 * IRQ numbers of Interrupts(reference taken from vector table of uC)
 * will update with the growth of project
 * */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

/*
 * base address of various memory peripherals in the microcontroller
 * note: reference taken from reference manual flash memory section
 * */
#define FLASH_BASEADDR	0x08000000U /*start address of flash memory*/
#define SRAM_BASEADDR	0x20000000U /*start addr of SRAM - 96KB*/
#define ROM_BASEADDR	0x1FFF0000U /*start addr of system memory*/

/*
 * base address of various bus domains of the microcontroller
 * note: reference taken from uC reference manual
 * */
#define PERI_BASEADDR		0x40000000U /*uC peripheral start addr*/
#define AHB1_BASEADDR		0x40020000U /*ahb1 bus peripheral start addr*/
#define AHB2_BASEADDR		0x50000000U /*ahb2 bus peripheral start addr*/
#define APB1_BASEADDR		PERI_BASEADDR /*apb1 bus peri atart addr*/
#define APB2_BASEADDR		0x40010000U /*apb2 bus peri start addr*/

/*
 * base address of peripherals hanging on AHB1 bus
 * took reference from uC reference manual
 * currently not defining all peripherals only the required peripherals for the project
 * */
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000U) /*GPIOA peri base addr, AHB1 base addr + gpioA offset*/
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400U) /*GPIOB peri base addr, AHB1 base addr + gpioB offset*/
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800U) /*GPIOC peri base addr, AHB1 base addr + gpioC offset*/
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00U) /*GPIOD peri base addr, AHB1 base addr + gpioD offset*/
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000U) /*GPIOE peri base addr, AHB1 base addr + gpioE offset*/
#define GPIOH_BASEADDR		(AHB1_BASEADDR + 0x1C00U) /*GPIOH peri base addr, AHB1 base addr + gpioH offset*/
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800U) /*RCC peri base addr, AHB1 base addr + RCC offset*/

/*
 * base address of peripherals hanging on APB1 bus
 * reference taken from uC reference manual
 * */
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800U) /*spi2 peri base address, apb1 base addr + spi2 base*/
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00U) /*spi3 peri base address, apb1 base addr + spi3 base*/
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400U) /*i2c1 peri base address, apb1 base addr + i2c1 base*/
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800U) /*i2c2 peri base address, apb1 base addr + i2c2 base*/
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00U) /*i2c3 peri base address, apb1 base addr + i2c3 base*/
#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400U) /*usart2 peri base address, apb1 base addr + usart2 base*/

/*
 * base addr of peripherals hanging from APB2 bus
 * reference taken from uC reference manual
 * */
#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000U) /*usart1 peri base address, apb2 base addr + usart1 base*/
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400U) /*usart6 peri base address, apb2 base addr + usart6 base*/
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000U) /*spi1 peri base address, apb2 base addr + spi1 base*/
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400U) /*spi4 peri base address, apb2 base addr + spi4 base*/
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800U) /*syscfg peri base address, apb2 base addr + syscfg base*/
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00U) /*exti peri base address, apb2 base addr + exti base*/

/*###################### peripheral register definition structures ##############################
 * - peripheral registers varies based on the MCU used
 * - eg. stm32f4/stm32l4/stm32f0 might have different number of registers of spi/i2c etc
 * - please refer to your uC for the correct peripheral registers
 * */

/*
 * peripheral register definition structure of GPIO
 * */
typedef struct {
	__vo uint32_t MODER; /*GPIO port mode register, address offset=0x00*/
	__vo uint32_t OTYPER; /*GPIO port output type register, address offset=0x04*/
	__vo uint32_t OSPEEDR; /*GPIO port output speed register, address offset=0x08*/
	__vo uint32_t PUPDR; /*GPIO port pull-up/pull-down register, address offset=0x0C*/
	__vo uint32_t IDR; /*GPIO port input data register, address offset=0x10*/
	__vo uint32_t ODR; /*GPIO port output data register, address offset=0x14*/
	__vo uint32_t BSRR; /*GPIO port bit set/reset register, address offset=0x18*/
	__vo uint32_t LCKR; /*GPIO port configuration lock register, address offset=0x1C*/
	__vo uint32_t AFR[2]; /*AFR[0] : GPIO alternate function low register address offset=0x20, AFR[1] : GPIO alternate function high register, address offset=0x24*/
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 * */
typedef struct {
	__vo uint32_t CR; /*RCC clock control register ,address offset=0x00*/
	__vo uint32_t PLLCFGR; /*RCC PLL configuration register, address offset=0x04*/
	__vo uint32_t CFGR; /*RCC clock configuration register, address offset=0x08*/
	__vo uint32_t CIR; /*RCC clock interrupt register, address offset=0x0C*/
	__vo uint32_t AHB1RSTR; /*RCC AHB1 peripheral reset register, address offset=0x10*/
	__vo uint32_t AHB2RSTR; /*RCC AHB2 peripheral reset register, address offset=0x14*/
	__vo uint32_t Reserved0[2]; /*Reserved0[0] : address offset=0x18, Reserved0[1] : address offset=0x1C*/
	__vo uint32_t APB1RSTR; /*RCC APB1 peripheral reset register, address offset=0x20*/
	__vo uint32_t APB2RSTR; /*RCC APB2 peripheral reset register, address offset=0x24*/
	__vo uint32_t Reserved1[2]; /*Reserved1[0]: addr offset=0x28, Reserved1[1] : addr offset=0x2C*/
	__vo uint32_t AHB1ENR; /*RCC AHB1 peripheral clock enable register, addr offset=0x30*/
	__vo uint32_t AHB2ENR; /*RCC AHB2 peripheral clock enable register, addr offset=0x34*/
	__vo uint32_t Reserved2[2]; /*Reserved2[0]: addr offset=0x38, Reserved2[1] : addr offset=0x3C*/
	__vo uint32_t APB1ENR; /*RCC APB1 peripheral clock enable register, addr offset=0x40*/
	__vo uint32_t APB2ENR; /*RCC APB2 peripheral clock enable register, addr offset=0x44*/
	__vo uint32_t Reserved3[2]; /*Reserved3[0]: addr offset=0x48, Reserved3[1] : addr offset=0x4C*/
	__vo uint32_t AHB1LPENR; /*RCC AHB1 peripheral clock enable in low power mode register, addr offset=0x50*/
	__vo uint32_t AHB2LPENR; /*RCC AHB2 peripheral clock enable in low power mode register, addr offset=0x54*/
	__vo uint32_t Reserved4[2]; /*Reserved4[0]: addr offset=0x58, Reserved4[1] : addr offset=0x5C*/
	__vo uint32_t APB1LPENR; /*RCC APB1 peripheral clock enable in low power mode register, addr offset=0x60*/
	__vo uint32_t APB2LPENR; /*RCC APB2 peripheral clock enabled in low power mode register, addr offset=0x64*/
	__vo uint32_t Reserved5[2]; /*Reserved5[0]: addr offset=0x68, Reserved5[1] : addr offset=0x6C*/
	__vo uint32_t BDCR; /*RCC Backup domain control register, addr offset=0x70*/
	__vo uint32_t CSR; /*RCC clock control & status register, addr offset=0x74*/
	__vo uint32_t Reserved6[2]; /*Reserved6[0]: addr offset=0x78, Reserved6[1] : addr offset=0x7C*/
	__vo uint32_t SSCGR; /*RCC spread spectrum clock generation register, addr offset=0x80*/
	__vo uint32_t PLLI2SCFGR; /*RCC PLLI2S configuration register, addr offset=0x84*/
	__vo uint32_t Reserved7; /*addr offset=0x88*/
	__vo uint32_t DCKCFGR; /*RCC Dedicated Clocks Configuration Register, addr offset=0x8C*/
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 * */
typedef struct {
	__vo uint32_t IMR; /*Interrupt mask register, address offset=0x00*/
	__vo uint32_t EMR; /*Event mask register, address offset=0x04*/
	__vo uint32_t RTSR; /*Rising trigger selection register, address offset=0x08*/
	__vo uint32_t FTSR; /*Falling trigger selection register, address offset=0x0C*/
	__vo uint32_t SWIER; /*Software interrupt event register, address offset=0x10*/
	__vo uint32_t PR; /*Pending register, address offset=0x14*/
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 * */
typedef struct {
	__vo uint32_t MEMRMP; /*SYSCFG memory remap register, address offset=0x00*/
	__vo uint32_t PMC; /*SYSCFG peripheral mode configuration register, address offset=0x04*/
	__vo uint32_t EXTICR[4]; /*SYSCFG external interrupt configuration register, EXTICR[0] : address offset=0x08, EXTICR[1] : address_offset=0x0C, EXTICR[2] : address_offset=0x10, EXTICR[3] : address_offset=0x14*/
	__vo uint32_t FTSR; /*Falling trigger selection register, address offset=0x0C*/
	__vo uint32_t Reserved[2]; /*Reserved[0] : address offset=0x18, Reserved[1] : address_offset=0x1C*/
	__vo uint32_t CMPCR; /*Compensation cell control register, address offset=0x20*/
}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure of GPIO
 * */
typedef struct {
	__vo uint32_t CR1; /*SPI control register 1, address offset=0x00*/
	__vo uint32_t CR2; /*SPI control register 2, address offset=0x04*/
	__vo uint32_t SR; /*SPI status register, address offset=0x08*/
	__vo uint32_t DR; /*SPI data register, address offset=0x0C*/
	__vo uint32_t CRCPR; /*SPI CRC polynomial register, address offset=0x10*/
	__vo uint32_t RXCRCR; /*SPI RX CRC register, address offset=0x14*/
	__vo uint32_t TXCRCR; /*SPI TX CRC register, address offset=0x18*/
	__vo uint32_t I2SCFGR; /*SPI_I2S configuration register, address offset=0x1C*/
	__vo uint32_t I2SPR; /*SPI_I2S prescaler register, address offset=0x20*/
}SPI_RegDef_t;

/*
 * peripheral definitions (peripheral base addressed typecasted to xxx_RegDef_t)
 * */

/*
 * peripheral definitions of GPIO
 * */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*
 * peripheral definitions of RCC
 * */
#define RCC		 	((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * peripheral definitions of EXIT
 * */
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * peripheral definitions of SYSCFG
 * */
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * peripheral definitions of SPI
 * */
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * peripheral clock enable and disable definitions
 * */
/*
 * Clock enable macros for GPIO
 * */
#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock enable macros for I2C
 */
#define I2C1_CLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPI
 * */
#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable marcos for USART
 * */
#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macro for SYSCFG
 * */
#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIO
 * */
#define GPIOA_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock disable macros for SPI
 * */
#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * CLock disable macros for I2C
 * */
#define I2C1_CLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * CLock disable macros for USART
 * */
#define USART1_CLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_CLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG
 * */
#define SYSCFG_CLK_DI()		(RCC->APB2ENR &= ~(1 << 14)))

/*
 * GPIO reset macros
 * */
#define GPIOA_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_RESET_REG()	do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * SPI reset macros
 * */
#define SPI1_RESET_REG()		do { (RCC->APB2RSTR) |= (1 << 12); (RCC->APB2RSTR) &= ~(1 << 12); }while(0)
#define SPI2_RESET_REG()		do { (RCC->APB1RSTR) |= (1 << 14); (RCC->APB1RSTR) &= ~(1 << 14); }while(0)
#define SPI3_RESET_REG()		do { (RCC->APB1RSTR) |= (1 << 15); (RCC->APB1RSTR) &= ~(1 << 15); }while(0)
#define SPI4_RESET_REG()		do { (RCC->APB2RSTR) |= (1 << 13); (RCC->APB2RSTR) &= ~(1 << 13); }while(0)

/*
 * GPIO port to EXTI code to put into EXTICR registers of STSCFG
 * */
#define GPIO_PORT_TO_CODE(x)		(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOH) ? 7 : 0

/*
 * Some miscellaneous macros
 * */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F401RE_H_ */
