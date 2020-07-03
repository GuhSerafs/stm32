/*
 * my_stm32f401re_gpio_driver.h
 *
 *  Created on: 27 de mar de 2020
 *      Author: Gustavo
 */

#ifndef INC_MY_STM32F401RE_GPIO_DRIVER_H_
#define INC_MY_STM32F401RE_GPIO_DRIVER_H_

#include <my_stm32f401re.h>

// Macros específicos do driver

// @GPIO_PIN_NUMBERS - Macros de identificação dos pinos
#define GPIO_PIN_NUM_0		0
#define GPIO_PIN_NUM_1		1
#define GPIO_PIN_NUM_2		2
#define GPIO_PIN_NUM_3		3
#define GPIO_PIN_NUM_4		4
#define GPIO_PIN_NUM_5		5
#define GPIO_PIN_NUM_6		6
#define GPIO_PIN_NUM_7		7
#define GPIO_PIN_NUM_8		8
#define GPIO_PIN_NUM_9		9
#define GPIO_PIN_NUM_10		10
#define GPIO_PIN_NUM_11		11
#define GPIO_PIN_NUM_12		12
#define GPIO_PIN_NUM_13		13
#define GPIO_PIN_NUM_14		14
#define GPIO_PIN_NUM_15		15

// @GPIO_PIN_MODES - Mode Macros
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

// @GPIO_PIN_OTYPER - Output Type Macros
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// @GPIO_PIN_OSPEEDR - Speed Macros
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// @GPIO_PIN_PUPDR - Pull up Pull down Macros
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

// Config Structure p/ um pino GPIO

typedef struct{
	uint8_t GPIO_PinNumber; 		// Número do pino a ser inicializado dentro da porta @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Modo do pino (MODER) @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// velocidade (OSPEEDR) @GPIO_PIN_OSPEEDR
	uint8_t GPIO_PinPuPdControl;	// PUPDR - @GPIO_PIN_PUPDR
	uint8_t GPIO_PinOPType;			// OTYPER - @GPIO_PIN_OTYPER
	uint8_t GPIO_PinAltFunMode;		// AFR
}GPIO_PinConfig_t;

// Handle Structure p/ um pino GPIO

typedef struct{
	GPIO_RegDef_t *pGPIOx; 				// Endereço da porto GPIO onde está o pino
	GPIO_PinConfig_t GPIO_PinConfig; 	//
}GPIO_Handler_t;

/*
 * Funções da API (Application Programming Interface)
 */

// Inicialização do clock
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

// Inicialização dos pinos
void GPIO_Init(GPIO_Handler_t *pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Leitura/escrita
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// Configuração de interrupções
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandler(uint8_t PinNumber);

#endif /* INC_MY_STM32F401RE_GPIO_DRIVER_H_ */
