/*
 * my_stm32f401re_rcc_driver.h
 *
 *  Created on: 17 de abr de 2020
 *      Author: Gustavo
 */

#ifndef INC_MY_STM32F401RE_RCC_DRIVER_H_
#define INC_MY_STM32F401RE_RCC_DRIVER_H_

#include <my_stm32f401re.h>

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLCLKValue(void);

#endif /* INC_MY_STM32F401RE_RCC_DRIVER_H_ */
