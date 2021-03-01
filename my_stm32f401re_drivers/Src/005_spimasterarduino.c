/*
 * 005_spimasterarduino.c
 *
 *  Created on: 7 de abr de 2020
 *      Author: Gustavo
 */

// SCK 		D13		PA5
// MISO		D12		PA6
// MOSI		D11		PA7
// NSS		D10		PA4
#include <my_stm32f401re_spi_driver.h>
#include <my_stm32f401re_gpio_driver.h>

#define SCK GPIO_PIN_NUM_5		// PA5
#define MISO GPIO_PIN_NUM_6		// PA6
#define MOSI GPIO_PIN_NUM_7		// PA7
#define NSS GPIO_PIN_NUM_4		// PA4

#define MODE_SPI 5

void delay(uint32_t c);

int main() {

	// Inicializar os clocks
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	SPI_PeriClockControl(SPI1, ENABLE);

	// Inicializar os pinos GPIO
	GPIO_Handler_t SPI_Pins_Config;

	// Inicialização do SCK (Output)
	SPI_Pins_Config.pGPIOx = GPIOA;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinAltFunMode = MODE_SPI;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = SCK;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull up pois não tem no sistema
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SPI_Pins_Config);

	/* MISO não é usado nessa aplicação */
	// Inicialização do MISO (Input, mas é configurado como alternate function)
	//SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = MISO;
	//GPIO_Init(&SPI_Pins_Config);

	// Inicialização do MOSI (mesmo...)
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = MOSI;
	GPIO_Init(&SPI_Pins_Config);

	// Inicialização do NSS (Aqui é output mesmo...)
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
	SPI_Config.SPI_Config.SPI_SSM = SPI_SSM_DISABLED; // Hardware Slave Select
	SPI_Config.SPI_Config.SPI_SSOE = SPI_SSOE_ENABLED; // Enable Slave Select Output Enable
	SPI_Config.SPI_Config.SPI_SclkSpeed = SPI_SCLK_PSCALER_DIV8; // Divisor de 256 p/ menor velocidade possível do sck
	SPI_Init(&SPI_Config);

	// Inicializar o botão do usuário
	GPIO_Handler_t BtnGPIO;
	BtnGPIO.pGPIOx = GPIOC;
	BtnGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BtnGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&BtnGPIO);

	// Buffer do programa
	char user_data[] = "Testando uma string maior";
	uint8_t data_len = sizeof(user_data);
	uint8_t trash = 0;

	while (1) {

		// Quando o usuário pressiona o botão, envia uma string
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13)) {
			// Habilitar o SPI
			SPI_DeviceControl(SPI1, ENABLE);

			// Enviar os dados
			SPI_DataSend(SPI1, &data_len, 1);
			SPI_DataSend(SPI1, (uint8_t*) user_data, sizeof(user_data));

			// Aguardar o Status sair de busy
			while (SPI1->SR & (1 << SPI_SR_BSY))
				;

			// Desativar o SPI
			SPI_DeviceControl(SPI1, DISABLE);

			// Liberar o escravo

			// Delay p filtrar
			delay(300);
		}

		if (SPI1->SR & (1 << SPI_SR_OVR)) { // Caso dê algum erro de overflow, reseta tudo aqui
			trash = SPI1->DR;
			trash = SPI1->SR;
		}

	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}
