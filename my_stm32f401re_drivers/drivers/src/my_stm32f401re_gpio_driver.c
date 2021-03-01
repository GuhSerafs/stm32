/*
 * my_stm32f401re_gpio_driver.c
 *
 *  Created on: 27 de mar de 2020
 *      Author: Gustavo
 */

#include <my_stm32f401re_gpio_driver.h>

/******************************************************
 * @fn                 	- GPIO_PeriClockControl
 *
 * @brief				- Habilita/Desabilita o clock de uma determinada porta GPIO.
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Enable/Disable Macros
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis) {
	if (EnOrDis == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

// Inicialização dos pinos

/******************************************************
 * @fn                 	- GPIO_Init
 *
 * @brief				- Faz a inicialização de um pino GPIO com seus parâmetros
 *
 * @param[in]			- Estrutura GPIO_Handler
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandler) {
	/* 1. Configurar modo do pino*************************
	 *
	 * O MODER possui 32 bits, com 2 bits para cada pino a partir do LSB
	 */
	uint32_t temp = 0;

	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) { // Verif. se não é um modo IT
		// Escrever o valor do PinMode no MODER
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER &= ~(0x3
				<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER |= temp;
	} else {
		// Modo de interrupção

		if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configurar o FTSR (Falling trigger selection register)
			// Clear FTSR e RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

			// Set FTSR
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. Configurar o RTSR (Rising trigger selection register)
			// Clear FTSR e RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

			// Set RTSR
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configurar ambos FTSR e RTSR

			// Set FTSR e RTSR
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configurar a porta GPIO no SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = 4 * (pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t gpio_val = GPIO_ADDR_TO_CODE(pGPIOHandler->pGPIOx);

		// Habilitar o CLOCK do SYSCFG

		SYSCFG_PCLK_EN();

		// Configurar o EXTICR apropriado
		SYSCFG->EXTICR[temp1] |= gpio_val << temp2;

		// 3. Habilitar a entrega da interrupção EXTI usando no IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configurar a velocidade do pino
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0x3
			<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Definir as configurações do Pull Up/Pull Down
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR &= ~(0x3
			<< (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configurar o tipo de saída
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configurar a função alternativa (se houver)
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF) {
		// Alternate function config...
		uint8_t temp1, temp2;
		temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 8;

		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		pGPIOHandler->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2);
		pGPIOHandler->pGPIOx->AFR[temp1] |= temp;
	}
}

/******************************************************
 * @fn                 	- GPIO_DeInit
 *
 * @brief				- Coloca a GPIO em reset state
 *
 * @param[in]			- Endereço da GPIO
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

// Leitura/escrita

/******************************************************
 * @fn                 	- GPIO_ReadFromInputPin
 *
 * @brief				- Lê o valor de um determinado pino GPIO
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Número do pino
 *
 *
 * @return				- booleano (uint8_t) [TRUE ou FALSE]
 *
 *
 * @note				- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
}

/******************************************************
 * @fn                 	- GPIO_ReadFromInputPort
 *
 * @brief				- Lê os 16 bits de uma porta GPIO
 *
 * @param[in]			- Endereço da GPIO
 *
 *
 * @return				- 16 bits (uint16_t)
 *
 *
 * @note				- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) pGPIOx->IDR;
}

/******************************************************
 * @fn                 	- GPIO_WriteToOutputPin
 *
 * @brief				- Escreve um valor em um determinado pino GPIO
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Número do Pino
 * @param[in] 			- Valor a ser escrito [booleano]
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/******************************************************
 * @fn                 	- GPIO_WriteToOutputPort
 *
 * @brief				- Escreve 16 bits em uma determinada porta GPIO
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Valor
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/******************************************************
 * @fn                 	- GPIO_ToggleOutputPin
 *
 * @brief				- Inverte o valor escrito em um dado pino GPIO
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Número do pino
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber); // 1 xor X = ~X
}

// Configuração de interrupções

/******************************************************
 * @fn                 	- GPIO_IRQConfig
 *
 * @brief				- ??
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Enable/Disable Macros
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis) {

	// 1. Config. IRQ Number
	if (EnOrDis) {
		if (IRQNumber <= 31) {
			NVIC->ISER[0] |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
			NVIC->ISER[1] |= (1 << IRQNumber % 32);
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
			NVIC->ISER[2] |= (1 << IRQNumber % 64);
		}
	} else {

		if (IRQNumber <= 31) {
			NVIC->ICER[0] |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
			NVIC->ICER[1] |= (1 << IRQNumber % 32);
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
			NVIC->ICER[2] |= (1 << IRQNumber % 64);
		}

	}

	// 2. Config. IRQPriority

	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;
	uint8_t shift = (8*IPRx_section) + 4; // OBs: 4 = nº de bits de prioridade implementados

	NVIC->IPR[IPRx] |= (IRQPriority) << shift;

}

/******************************************************
 * @fn                 	- GPIO_IRQHandler
 *
 * @brief				- ??
 *
 * @param[in]			- Endereço da GPIO
 * @param[in]			- Enable/Disable Macros
 *
 *
 * @return				- none
 *
 *
 * @note				- none
 */
void GPIO_IRQHandler(uint8_t PinNumber){
	// Limpar o Pending Register do EXTI
	// A limpeza é feita escrevendo 1 no registrador, cf. user manual.
	if (EXTI->PR & ( 1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
