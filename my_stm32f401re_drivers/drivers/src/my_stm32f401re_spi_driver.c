/*
 * my_stm32f401re_spi_driver.c
 *
 *  Created on: 2 de abr de 2020
 *      Author: Gustavo
 */

#include <my_stm32f401re_spi_driver.h>

/*
 *
 * 					IMPLEMENTAÇÃO DOS PROTÓTIPOS DAS API SUPORTADAS
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis) {
	if (EnOrDis) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handler_t *pSPIHandler) {
	uint32_t temp = 0;

	// Config SPI Mode
	pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_MSTR); 				// Clear bit
	temp = (pSPIHandler->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR); // Load reg
	pSPIHandler->pSPIx->CR1 |= temp; 						// Set Mode
	temp = 0; 												// Clear reg

	// Config SPI BusConfig
	if (pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_FULL_DUPLEX) {
		pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE); // Clear BIDIMODE bit
	} else if (pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_HALF_DUPLEX) {
		pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE); // Set BIDIMODE bit
	} else if (pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_SIMPLEX_RX) {
		pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE); // Clear BIDIMODE bit
		pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);	// Set RXONLY bit
	}

	// Config SSM and SSI
	if (pSPIHandler->SPI_Config.SPI_SSM == SPI_SSM_ENABLED
			&& pSPIHandler->SPI_Config.SPI_DeviceMode == SPI_MODE_MASTER) {
		pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_SSM);
		pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_SSM);
	}

	// Config SPI_SclkSpeed, DFF, CPOL, CPHA and SSM
	temp = (pSPIHandler->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR)
			| (pSPIHandler->SPI_Config.SPI_DFF << SPI_CR1_DFF)
			| (pSPIHandler->SPI_Config.SPI_CPOL << SPI_CR1_CPOL)
			| (pSPIHandler->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandler->pSPIx->CR1 |= temp;

	// Config SSOE
	if (pSPIHandler->SPI_Config.SPI_SSM == SPI_SSM_DISABLED
			&& pSPIHandler->SPI_Config.SPI_DeviceMode == SPI_MODE_MASTER) {
		if (pSPIHandler->SPI_Config.SPI_SSOE == SPI_SSOE_ENABLED) {
			pSPIHandler->pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		} else {
			pSPIHandler->pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
	}
}

void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_DeviceControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis) {
	if (EnOrDis) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); // Habilita o SPI Enable
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); // Desativa o SPI Enable
	}
}

// Send and Rcv Data

void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen) {
	uint32_t Len = dataLen;
	while (dataLen > 0) {
		while (!(pSPIx->SR & (1 << SPI_SR_TXE)))
			; // Aguarda o TXE ficar vazio.
		// Supondo 8 bits
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits
			pSPIx->DR = *((uint16_t*) (pTxBuffer + Len - dataLen));
			//(uint16_t*) pTxBuffer++;
			dataLen--;

		} else {
			// 8 bits
			pSPIx->DR = *(pTxBuffer + Len - dataLen);
			//pTxBuffer++;
		}
		dataLen--;
	}
}

void SPI_DataRecv(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen) {
	uint32_t Len = dataLen;
	while (dataLen > 0) {
		while (!(pSPIx->SR & (1 << SPI_SR_RXNE))) // Espera o Rx ficar cheio
			;
		// Supondo 8 bits
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits
			*((uint16_t*) (pRxBuffer + Len - dataLen)) = pSPIx->DR;
			//(uint16_t*) pTxBuffer++;
			dataLen--;

		} else {
			// 8 bits
			*(pRxBuffer + Len - dataLen) = pSPIx->DR;
			//pTxBuffer++;
		}
		dataLen--;
	}
}

// IRQ Config and Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void SPI_IRQHandler(SPI_Handler_t *pSPIHandler);
