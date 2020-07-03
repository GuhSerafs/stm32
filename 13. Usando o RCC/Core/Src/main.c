// Feito por Gustavo D. Serafim
// Baseado no curso "Mastering Microcontroller

// O objetivo do código é habilitar/desabilitar o clock do periférico ADC e GPIO
// Obs: Led fica no GPIOA pino 5


#include <stdint.h>
#include <stm32f4xx.h>

int main(){
	ADC_TypeDef *pADC; // Ponteiro do tipo ADC_TypeDef
	RCC_TypeDef *pRCC;
	GPIO_TypeDef *pGPIO;

	pADC = ADC1; // Atribuindo o endereço do ADC1 para o ponteiro
	pRCC = RCC; // Atribuindo o endereço do RCC para o ponteiro
	pGPIO = GPIOA; // Atribuindo o endereço do GPIOA para o ponteiro

	// Habilitando o Clock do ADC
	// Como o ADC1 está conectado ao APB2, devemos modificar o RCC_APB2ENR
	// Bit 8 do RCC ativa o clock do ADC
	pRCC->APB2ENR = (pRCC->APB2ENR | (1 << 8));

	// Como o GPIOA está no AHB1, temos que habilitar o RCC_AHB1ENR
	// GPIOA está no endereço 0

	pRCC->AHB1ENR = (pRCC->AHB1ENR | (1 << 0));

	// Modificando os registradores
	pADC->CR1 = 0x55; // CR1 é o "Control Register" do ADC [ Apenas para testar ]
	pGPIO->PUPDR = 0x11; // PUPDR é o "Pull Up/Down Register" do GPIO [Apenas para testar ]

	return 0;
}
