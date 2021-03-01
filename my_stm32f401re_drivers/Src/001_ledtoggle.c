/*	Data: 27/03/2020
 *	Autor: Gustavo D. Serafim
 * 	Objetivos: Testar o driver GPIO criado
 * 	Exercício: Inicializar o pino do LED em PushPull e Open Drain e fazê-lo piscar.
 *
 */

#include <my_stm32f401re.h>
#include <my_stm32f401re_gpio_driver.h>

void delay(uint32_t c);

int main() {

	// Inicialização da estrutura de configuração da GPIO
	GPIO_Handler_t LedGPIO;

	LedGPIO.pGPIOx = GPIOA;
	LedGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LedGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	LedGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LedGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LedGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/*	Push Pull + No PuPd 	-> Led pisca em intensidade normal
	 *  Open Drain + No PuPd 	-> Led não pisca (sem resistor de PU)
	 *  Open Drain + Pu 		-> Led Pisca em baixa intensidade (necessita PU externo)
	 */

	// Inicialização dos clocks
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&LedGPIO);

	while (1) {
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay(1000);
	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}
