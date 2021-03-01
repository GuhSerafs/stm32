/*
 * 004_spitxtest.c
 *
 *  Created on: 2 de abr de 2020
 *      Author: Gustavo
 */

// SCK 		D13		PA5
// MISO		D12		PA6
// MOSI		D11		PA7
// NSS		D10		PA4
#include <my_stm32f401re_spi_driver.h>
#include <my_stm32f401re_gpio_driver.h>

#define SCK GPIO_PIN_NUM_5
#define MISO GPIO_PIN_NUM_6
#define MOSI GPIO_PIN_NUM_7
#define NSS GPIO_PIN_NUM_4

#define MODE_SPI 5

int main() {

	// Inicializar os clocks
	GPIO_PeriClockControl(GPIOA, ENABLE);
	SPI_PeriClockControl(SPI1, ENABLE);

	// Inicializar os pinos GPIO
	GPIO_Handler_t SPI_Pins_Config;

	SPI_Pins_Config.pGPIOx = GPIOA;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinAltFunMode = MODE_SPI;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = SCK;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SPI_Pins_Config);

	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = MISO;
	GPIO_Init(&SPI_Pins_Config);

	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = MOSI;
	GPIO_Init(&SPI_Pins_Config);

	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = NSS;
	GPIO_Init(&SPI_Pins_Config);

	// Inicializar o SPI
	SPI_Handler_t SPI_Config;

	SPI_Config.pSPIx = SPI1;
	SPI_Config.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI_Config.SPI_Config.SPI_CPHA = SPI_CPHA_1ST_EDGE;
	SPI_Config.SPI_Config.SPI_CPOL = SPI_CPOL_LOW_IDLE;
	SPI_Config.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	SPI_Config.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI_Config.SPI_Config.SPI_SSM = SPI_SSM_ENABLED;
	SPI_Config.SPI_Config.SPI_SclkSpeed = SPI_SCLK_PSCALER_DIV256;
	SPI_Init(&SPI_Config);

	// Habilitar o dispositivo do SPI
	SPI_DeviceControl(SPI1, ENABLE);

	// Programa
	uint8_t user_data[] = "Hell oh World";
	SPI_DataSend(SPI1, user_data, sizeof(user_data));
	while(1);
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}
