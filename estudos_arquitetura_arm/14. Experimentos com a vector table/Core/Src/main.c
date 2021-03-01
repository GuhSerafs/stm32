#include <stdint.h>

#define BASE_SRAM_ADDR 0x20000000

const char my_data[] = "Testando o NVIC";

int main(){
	for(int i = 0; i<sizeof(my_data); i++){
		*(uint8_t*)(BASE_SRAM_ADDR + i) = my_data[i];
	}

	return 0;
}

/* Na tabela de vetores, o NMI fica no endereço 0x0000 0008 e o USART2 fica no endereço 0x0000 00D8
 * No arquivo startup_stm32f401retx.s estão os protótipos das funções que são "sobrepostas" aqui
 * Ao sobrepor essas funções, o endereço da função é escrito no endereço no NVIC.
 * Obs: o endereço no NVIC é acrescido de 1 & para ver é preciso criar um monitor de memória
 * em Hex Integer
 */

void NMI_Handler(void){

}

void USART2_IRQHandler(void){

}
