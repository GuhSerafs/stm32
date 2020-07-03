/*
 * my_stm32f401re.h
 *
 *  Created on: Mar 25, 2020
 *      Author: Gustavo
 */

#ifndef INC_MY_STM32F401RE_H_
#define INC_MY_STM32F401RE_H_

#include <stdint.h>

/*
 * Macros de propósito geral
 */

#define __vo volatile
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

// Base ADDRs da Flash, SRAM e NVIC
#define FLASH_BASEADDR  	0x08000000U // sem o U, o compilador entenderia como signed int
#define SRAM1_BASEADDR  	0x20000000U // 96kB
//#define SRAM2_BASEADDR      0x20018000U // provavelmente não tem nessa mcu
#define ROM_BASEADDR		0x1FFF0000 // System Memory
#define SRAM 				SRAM1_BASEADDR
#define NVIC_BASEADDR		0xE000E100U

// Base ADDRs do APBx e do AHBx
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

// Base ADDRs dos periféricos do AHB1 (Apenas GPIO)
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800)

// Base ADDRs dos periféricos do APB1 (I2C, SPI e USART)
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

// Base ADDRs dos periféricos do APB2 (USART, EXTI e SYSCFG)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)

// Definição das linhas de EXTI
#define IRQ_NO_EXTI0		6
#define	IRQ_NO_EXTI1		7
#define	IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define	IRQ_NO_EXTI4		10
#define	IRQ_NO_EXTI9_5		23
#define	IRQ_NO_EXTI15_10	40
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART6		71

/*
 * Definição das Estruturas dos Periféricos
 */

// NVIC

typedef struct{
	__vo uint32_t ISER[8];
	uint32_t RESERVED0[24];
	__vo uint32_t ICER[8];
	uint32_t RESERVED1[24];
	__vo uint32_t ISPR[8];
	uint32_t RESERVED2[24];
	__vo uint32_t ICPR[8];
	uint32_t RESERVED3[24];
	__vo uint32_t IABR[8];
	uint32_t RESERVED4[56];
	__vo uint32_t IPR[60];
}NVIC_RegDef_t;


//RCC
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RST2;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

//SYSCFG
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

//EXTI
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

//GPIO
typedef struct{
	__vo uint32_t MODER;		// Mode Register
	__vo uint32_t OTYPER;		// Output Type Register
	__vo uint32_t OSPEEDR;		// Output Speed Register
	__vo uint32_t PUPDR;		// Pull Up Pull Down Register
	__vo uint32_t IDR;			// Input Data Register
	__vo uint32_t ODR;			// Output Data Register
	__vo uint32_t BSRR;			// Bit Set Reset Register
	__vo uint32_t LCKR;			// Lock Register
	__vo uint32_t AFR[2];		// Alternate Function Registers [0] -> Low; [1] -> High
}GPIO_RegDef_t;

//SPI
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

//I2C
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

//USART
typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*
 * Definição dos periféricos em forma de ponteiro para as estruturas de registradores
 */

//NVIC
#define NVIC ((NVIC_RegDef_t*)NVIC_BASEADDR)
//RCC
#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

//SYSCFG
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

//EXTI
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

//GPIO
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

//SPI
#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASEADDR)

//I2C
#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASEADDR)


//USART
#define USART1 ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6 ((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Macros dos bits de registradores dos periféricos
 */

// Bits do RCC CFGR
#define RCC_CFGR_SW0		0
#define RCC_CFGR_SW1		1
#define RCC_CFGR_SWS0		2
#define RCC_CFGR_SWS1		3
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_PPRE1		10
#define RCC_CFGR_PPRE2		13
#define RCC_CFGR_RTCPRE		16
#define RCC_CFGR_MCO1		21
#define RCC_CFGR_I2SSCR		23
#define RCC_CFGR_MCO1PRE	24
#define RCC_CFGR_MCO2PRE	27
#define RCC_CFGR_MCO2		30

// Bits do SPI_CR1
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

// Bits do SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

// Bits do SPI_SR
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8


// Bits do USART_SR
#define USART_SR_PE		0
#define USART_SR_FE 	1
#define USART_SR_NF 	2
#define USART_SR_ORE 	3
#define USART_SR_IDLE 	4
#define USART_SR_RXNE 	5
#define USART_SR_TC 	6
#define USART_SR_TXE 	7
#define USART_SR_LBD 	8
#define USART_SR_CTS 	9

// Bits do USART_BRR
#define USART_BRR_DIVFRACT	0
#define USART_BRR_MANTISSA 	4

// Bits do USART_CR1
#define USART_CR1_SBK 		0
#define USART_CR1_RWU 		1
#define USART_CR1_RE 		2
#define USART_CR1_TE 		3
#define USART_CR1_IDLEIE 	4
#define USART_CR1_RXNEIE 	5
#define USART_CR1_TCIE 		6
#define USART_CR1_TXEIE 	7
#define USART_CR1_PEIE 		8
#define USART_CR1_PS 		9
#define USART_CR1_PCE 		10
#define USART_CR1_WAKE 		11
#define USART_CR1_M 		12
#define USART_CR1_UE 		13
#define USART_CR1_OVER8 	15

// Bits do USART_CR2
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

// Bits do USART_CR3
#define USART_CR3_EIE		0
#define USART_CR3_IREN 		1
#define USART_CR3_IRLP 		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK 		4
#define USART_CR3_SCEN 		5
#define USART_CR3_DMAR 		6
#define USART_CR3_DMAT 		7
#define USART_CR3_RTSE 		8
#define USART_CR3_CTSE 		9
#define USART_CR3_CTSIE 	10
#define USART_CR3_ONEBIT 	11

/*
 * Clock ENABLE e DISABLE para os periféricos
 */

// SYSCFG ENABLE/DISABLE

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))


// GPIO ENABLE
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

// GPIO DISABLE
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

// GPIO REG RESET
#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)

// GPIO ADDR TO CODE

#define GPIO_ADDR_TO_CODE(x)	((x == GPIOA)? 0 : \
								(x == GPIOB)? 1 : \
								(x == GPIOC)? 2 : \
								(x == GPIOD)? 3 : \
								(x == GPIOE)? 4 : \
								(x == GPIOH)? 7 : -1)

//I2C ENABLE
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

//I2C DISABLE
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

//SPI ENABLE
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

//SPI DISABLE
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

//USART ENABLE
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

//USART DISABLE
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))


#endif /* INC_MY_STM32F401RE_H_ */
