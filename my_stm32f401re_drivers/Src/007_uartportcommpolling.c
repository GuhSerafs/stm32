/*
 * 007_uartportcommpolling.c
 *
 *  Created on: 17 de abr de 2020
 *      Author: Gustavo
 */

#include <my_stm32f401re_usart_driver.h>
#include <my_stm32f401re_gpio_driver.h>

#define USART_TX GPIO_PIN_NUM_2		// PA2
#define USART_RX GPIO_PIN_NUM_3		// PA3
#define BTN_PIN GPIO_PIN_NUM_13		// PC13

#define MODE_USART 7

#define CR 0x0D
#define LF 0x0A

extern void initialise_monitor_handles(void);

void delay(uint32_t c);

char txBuffer[] = "___Hello World - Aguardando mensagem...";
char rcvBuffer[255];
char echoBuffer[255];

int main() {

	//initialise_monitor_handles();

	// Inicializar os clocks
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	USART_PeriClockControl(USART2, ENABLE);

	// Inicializar os pinos GPIO
	GPIO_Handler_t USART_Pins_Config;
	GPIO_Handler_t Btn_Pin_Config;

	// Inicialização do botão
	Btn_Pin_Config.pGPIOx = GPIOC;
	Btn_Pin_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Btn_Pin_Config.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
	Btn_Pin_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Btn_Pin_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // Pull up pois não tem no sistema
	Btn_Pin_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&Btn_Pin_Config);

	// Inicialização do TX
	USART_Pins_Config.pGPIOx = GPIOA;
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinAltFunMode = MODE_USART;
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = USART_TX;
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull up pois não tem no sistema
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&USART_Pins_Config);

	//Inicialização do RX
	USART_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = USART_RX;
	GPIO_Init(&USART_Pins_Config);

	// Inicializar o USART
	USART_Handle_t USART2_Params;

	USART2_Params.pUSARTx = USART2;
	USART2_Params.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_Params.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_Params.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART2_Params.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_Params.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2_Params.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2_Params);

	// Inicializar o botão do usuário
	GPIO_Handler_t BtnGPIO;
	BtnGPIO.pGPIOx = GPIOC;
	BtnGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BtnGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&BtnGPIO);

	USART_PeripheralControl(USART2, ENABLE);
	USART_SendData(&USART2_Params, (uint8_t*) txBuffer, sizeof(txBuffer));
	USART_PeripheralControl(USART2, DISABLE);

	while (1) {
		// ECHO COM POLLING

		// Recebe
		int i = -1;
		USART_PeripheralControl(USART2, ENABLE);
		do {
			i++;
			USART_ReceiveData(&USART2_Params, (uint8_t*) (rcvBuffer + i), 1);
			rcvBuffer[i] = rcvBuffer[i];
		} while (rcvBuffer[i] != LF);

		// Envia o que recebeu
		USART_SendData(&USART2_Params, (uint8_t*)rcvBuffer, ++i);
		USART_PeripheralControl(USART2, DISABLE);
	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}
