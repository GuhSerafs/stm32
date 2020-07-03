/*
 * 006_spimasterarduinoack.c
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

// Códigos de comando
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

#define ACK 0xF5

// Localização do pino do LED no arduino
#define LED 9

extern void initialise_monitor_handles(void);

void delay(uint32_t c);
int SPI_VerifyResponse(uint8_t ackbyte);
void SPI_WaitBsy(void);

int main() {

	initialise_monitor_handles();

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

	//Inicialização do MISO (Input, mas é configurado como alternate function)
	SPI_Pins_Config.GPIO_PinConfig.GPIO_PinNumber = MISO;
	GPIO_Init(&SPI_Pins_Config);

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
	SPI_Config.SPI_Config.SPI_SclkSpeed = SPI_SCLK_PSCALER_DIV16; // Divisor de 256 p/ menor velocidade possível do sck
	SPI_Init(&SPI_Config);

	// Inicializar o botão do usuário
	GPIO_Handler_t BtnGPIO;
	BtnGPIO.pGPIOx = GPIOC;
	BtnGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	BtnGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&BtnGPIO);

	// Buffer do programa
	uint8_t buffer_recv[11];
	uint8_t buffer_transfer[] = "Vou enviar uma mensagem grandinha";
	uint8_t cmd_byte;
	uint8_t resp_byte;
	uint8_t args[2];
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	while (1) {

		// Enviar o comando COMMAND_LED_CTRL
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			// Aguarda o botão ser pressionado
			;
		delay(500);

		// Enviar o comando COMMAND_LED_CTRL
		cmd_byte = COMMAND_LED_CTRL;

		SPI_DeviceControl(SPI1, ENABLE); 		// Habilita o dispositivo (SPI)

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o buffer

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para carregar o buffer
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Enviar os argumentos
			args[0] = LED;
			args[1] = LED_ON;

			SPI_WaitBsy();
			SPI_DataSend(SPI1, args, sizeof(args)); // Escrita postiça para carregar o buffer
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

			printf("COMMAND_LED_CTRL Executado\n");
		}

		// Enviar o comando COMMAND_SENSOR_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			// Aguarda o botão ser pressionado
			;
		delay(500);

		cmd_byte = COMMAND_SENSOR_READ;

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Enviar os argumentos
			args[0] = ANALOG_PIN0;

			SPI_WaitBsy();
			SPI_DataSend(SPI1, args, 1); // Envia os argumentos
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

			delay(100);
			uint8_t analog_read;
			SPI_WaitBsy();
			SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &analog_read, sizeof(analog_read)); // Leitura do buffer

			printf("Leitura analógica: %d\n", analog_read);
		}

		// Enviar o comando COMMAND_LED_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			;
		delay(500);

		cmd_byte = COMMAND_LED_READ;

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Enviar os argumentos
			args[0] = LED;

			SPI_WaitBsy();
			SPI_DataSend(SPI1, args, 1); // Envia os argumentos
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

			delay(100);
			uint8_t digital_read;
			SPI_WaitBsy();
			SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &digital_read, sizeof(digital_read)); // Leitura do buffer

			printf("Leitura digital: %d\n", digital_read);
		}

		// Enviar o comando COMMAND_PRINT
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			;
		delay(500);

		cmd_byte = COMMAND_PRINT;

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Enviar os argumentos
			args[0] = sizeof(buffer_transfer);

			SPI_WaitBsy();
			SPI_DataSend(SPI1, args, 1); // Envia os argumentos
			SPI_WaitBsy();
			SPI_DataSend(SPI1, buffer_transfer, sizeof(buffer_transfer)); // Leitura postiça pra limpar o RXNE
			printf("COMMAND_PRINT Executado\n");
		}

		// Enviar o comando COMMAND_ID_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			;
		delay(2000);

		cmd_byte = COMMAND_ID_READ;

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para limpar o TXE
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Receber a board ID
			for (int i = 0; i < 10; i++) {
				SPI_WaitBsy();
				SPI_DataSend(SPI1, &dummy_write, 1);
				SPI_WaitBsy();
				SPI_DataRecv(SPI1, &buffer_recv[i], 1);
			}
			buffer_recv[10] = '\0';

			printf("ID Recebido: %s\n", buffer_recv);
		}

		// Enviar o comando COMMAND_LED_CTRL
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))
			// Aguarda o botão ser pressionado
			;
		delay(500);

		// Enviar o comando COMMAND_LED_CTRL
		cmd_byte = COMMAND_LED_CTRL;

		SPI_DeviceControl(SPI1, ENABLE); 		// Habilita o dispositivo (SPI)

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &cmd_byte, sizeof(cmd_byte)); // Escrita no buffer p/ enviar
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o buffer

		SPI_WaitBsy();
		SPI_DataSend(SPI1, &dummy_write, sizeof(dummy_write)); // Escrita postiça para carregar o buffer
		SPI_WaitBsy();
		SPI_DataRecv(SPI1, &resp_byte, sizeof(resp_byte)); 	// Leitura do buffer

		printf("Resposta: %#x\n", resp_byte);

		// Verificar se o comando é suportado
		if (SPI_VerifyResponse(resp_byte)) {
			// Enviar os argumentos
			args[0] = LED;
			args[1] = LED_OFF;

			SPI_WaitBsy();
			SPI_DataSend(SPI1, args, sizeof(args)); // Escrita postiça para carregar o buffer
			SPI_WaitBsy();
			SPI_DataRecv(SPI1, &dummy_read, sizeof(dummy_read)); // Leitura postiça pra limpar o RXNE
			printf("COMMAND_LED_CTRL Executado\n");
		}

		SPI_WaitBsy();
		SPI_DeviceControl(SPI1, DISABLE); 		// Desativa o dispositivo (SPI)

	}
}

void delay(uint32_t c) {

	for (int j = 0; j < c; j++) {
		for (uint32_t i = 0; i < 720; i++) {
		}
	}
}

int SPI_VerifyResponse(uint8_t ackbyte) {
	if (ackbyte == ACK) {
		return 1;
	} else {
		return 0;
	}
}

void SPI_WaitBsy(void) {
	while (SPI1->SR & (1 << SPI_SR_BSY))
		;	// Aguarda sair de BSY
	return;
}
