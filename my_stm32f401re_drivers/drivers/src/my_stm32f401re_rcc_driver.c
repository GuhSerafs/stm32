/*
 * my_stm32f401re_rcc_driver.c
 *
 *  Created on: 17 de abr de 2020
 *      Author: Gustavo
 */

#include <my_stm32f401re_rcc_driver.h>

#define SWS_HSI 0
#define SWS_HSE 1
#define SWS_PLL 2

const uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
const uint8_t APB_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk, SystemClk;
	uint8_t clksrc, temp, ahb_pre, apb_pre;

	// Verificar o SWS para saber a fonte do clock do sistema
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if (clksrc == SWS_HSI){
		SystemClk = 16000000;
	}else if(clksrc == SWS_HSE){
		SystemClk = 8000000;
	}else if(clksrc == SWS_PLL){
		SystemClk = RCC_GetPLLCLKValue();
	}

	// Verificar o divisor do AHB prescaler
	temp = 0;
	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;

	if (temp < 8){
		ahb_pre = 1;
	}else{
		ahb_pre = AHB_PreScaler[temp-8];
	}

	// Verificar o divisor do APB
	temp = 0;
	temp = (RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7;

	if(temp < 4){
		apb_pre = 1;
	}else{
		apb_pre = APB_PreScaler[temp - 4];
	}

	pclk = SystemClk/(apb_pre * ahb_pre);

	return pclk;
}

uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk, SystemClk;
	uint8_t clksrc, temp, ahb_pre, apb_pre;

	// Verificar o SWS para saber a fonte do clock do sistema
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if (clksrc == SWS_HSI){
		SystemClk = 16000000;
	}else if(clksrc == SWS_HSE){
		SystemClk = 8000000;
	}else if(clksrc == SWS_PLL){
		SystemClk = RCC_GetPLLCLKValue();
	}

	// Verificar o divisor do AHB prescaler
	temp = 0;
	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;

	if (temp < 8){
		ahb_pre = 1;
	}else{
		ahb_pre = AHB_PreScaler[temp-8];
	}

	// Verificar o divisor do APB
	temp = 0;
	temp = (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7;

	if(temp < 4){
		apb_pre = 1;
	}else{
		apb_pre = APB_PreScaler[temp - 4];
	}

	pclk = SystemClk/(apb_pre * ahb_pre);

	return pclk;
}

uint32_t RCC_GetPLLCLKValue(void){
	return 0;
}
