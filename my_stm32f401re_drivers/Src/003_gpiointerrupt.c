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
	BtnGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	BtnGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	BtnGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	BtnGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BtnGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// OTYPER, OSPEEDR e PUPDR não faz diferença.

	// Inicialização dos clock
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&LedGPIO);
	GPIO_Init(&BtnGPIO);

	// Configuração da interrupção
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, 15, ENABLE);

	while (1) {
		// Nada, huehue

	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}


void EXTI15_10_IRQHandler(void){
	// Debounce
	delay(10);

	// Limpar o PR
	GPIO_IRQHandler(GPIO_PIN_NUM_13);

	// Tarefa:
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
}
