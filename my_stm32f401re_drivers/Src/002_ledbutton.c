/*
 * 002_ledbutten.c
 *
 *  Criado em: 27 de mar de 2020
 *      Autor: Gustavo D. Serafim
 *      Objetivos: Testar o driver GPIO criado
 * 		Exercício: Inicializar o pino do LED em PushPull e Open Drain e fazê-lo piscar.
 */

#include <my_stm32f401re.h>
#include <my_stm32f401re_gpio_driver.h>

void delay(uint32_t c);

int main() {

	// Inicialização da estrutura de configuração da GPIO
	GPIO_Handler_t LedGPIO;
	GPIO_Handler_t BtnGPIO;

	LedGPIO.pGPIOx = GPIOA;
	LedGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LedGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	LedGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LedGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LedGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	BtnGPIO.pGPIOx = GPIOC;
	BtnGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BtnGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;

	// OTYPER, OSPEEDR e PUPDR não faz diferença.

	// Inicialização dos clocks
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&LedGPIO);
	GPIO_Init(&BtnGPIO);

	while (1) {
		static uint8_t btn_state = 0;

		if (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13)) {
			if (btn_state == 0) btn_state = 1;
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
			delay(1000);
		}else if(btn_state == 1){
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_5, RESET);
			btn_state = 0;
		}

	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}
