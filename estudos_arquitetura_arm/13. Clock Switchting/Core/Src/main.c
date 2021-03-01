#include <stdint.h>
#include <stm32f4xx.h>

// O objetivo será modificar a fonte do clock para o HSE
// High Speed External (Source)

int main(){
	// Se verificarmos os registradores do RCC, vemos que
	// O padrão do RCC->RC é 0x83 (significa que o padrão é usar o HSI)

	// Então, precisamos:
	// 1. Ligar o HSE
	RCC_TypeDef *pRCC = RCC;
	pRCC->CR |= (1 << 16); // Bit 16 do Control Register do RCC habilita o HSE

	while( !(pRCC->CR & (1 << 17)) ); // Aguarda até o bit 17 (HSERDY) se tornar 1.

	// 2. Habilitar o HSE como fonte de clock
	pRCC->CFGR &= ~(0x3 << 0); // Reseta os 2 últimos bits do CFGR
	pRCC->CFGR |= (0x1 << 0); // Escreve 1 nos bits do CFGR

	return 0;
}
