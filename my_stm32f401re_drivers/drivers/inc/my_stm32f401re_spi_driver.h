/*
 * my_stm32f401re_spi_driver.h
 *
 *  Created on: 2 de abr de 2020
 *      Author: Gustavo
 */

#ifndef INC_MY_STM32F401RE_SPI_DRIVER_H_
#define INC_MY_STM32F401RE_SPI_DRIVER_H_

#include <my_stm32f401re.h>

// @SPI_DeviceMode
#define SPI_MODE_MASTER		1
#define SPI_MODE_SLAVE		0

// @SPI_BusConfig
#define SPI_BUS_FULL_DUPLEX		1   // 2-line unidirecional, output enabled
#define SPI_BUS_HALF_DUPLEX		2	// 1-line bidirecional, output enabled
#define SPI_BUS_SIMPLEX_RX		3	// 1-line unidirecional, output disabled

// @SPI_SclkSpeed
#define SPI_SCLK_PSCALER_DIV2		0
#define SPI_SCLK_PSCALER_DIV4		1
#define SPI_SCLK_PSCALER_DIV8		2
#define SPI_SCLK_PSCALER_DIV16		3
#define SPI_SCLK_PSCALER_DIV32		4
#define SPI_SCLK_PSCALER_DIV64		5
#define SPI_SCLK_PSCALER_DIV128		6
#define SPI_SCLK_PSCALER_DIV256		7

// @SPI_DFF
#define SPI_DFF_8BIT		0
#define SPI_DFF_16BIT		1

// @SPI_CPOL
#define SPI_CPOL_LOW_IDLE		0
#define SPI_CPOL_HIGH_IDLE		1

// @SPI_CPHA
#define SPI_CPHA_1ST_EDGE	0
#define SPI_CPHA_2ND_EDGE	1

// @SPI_SSM
#define SPI_SSM_DISABLED	0
#define SPI_SSM_ENABLED		1

// @SPI_SSOE
#define SPI_SSOE_DISABLED	0
#define SPI_SSOE_ENABLED	1

// Config Structure p/ um periférico SPI

typedef struct{
	uint8_t SPI_DeviceMode; 		// Número do pino a ser inicializado dentro da porta @GPIO_PIN_NUMBERS
	uint8_t SPI_BusConfig;			// Modo do pino (MODER) @GPIO_PIN_MODES
	uint8_t SPI_SclkSpeed;			// velocidade (OSPEEDR) @GPIO_PIN_OSPEEDR
	uint8_t SPI_DFF;	// PUPDR - @GPIO_PIN_PUPDR
	uint8_t SPI_CPOL;			// OTYPER - @GPIO_PIN_OTYPER
	uint8_t SPI_CPHA;		// AFR
	uint8_t SPI_SSM;
	uint8_t SPI_SSOE;
}SPI_Config_t;

// Handle Structure p/ um pino GPIO

typedef struct{
	SPI_RegDef_t *pSPIx; 				// Endereço da porto GPIO onde está o pino
	SPI_Config_t SPI_Config; 	//
}SPI_Handler_t;

/*
 *
 * 					IMPLEMENTAÇÃO DOS PROTÓTIPOS DAS API SUPORTADAS
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_DeviceControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

// Send and Rcv Data

void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen);
void SPI_DataRecv(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen);

// IRQ Config and Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void SPI_IRQHandler(SPI_Handler_t *pSPIHandler);

#endif /* INC_MY_STM32F401RE_SPI_DRIVER_H_ */
