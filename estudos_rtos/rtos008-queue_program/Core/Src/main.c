/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * PRECISEI MUDAR O cmsis_os.c NA LINHA 612 p/ LIMPAR OS BITS DA osSignalWait
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FreeRTOS.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId cmd_handling_taHandle;
osThreadId menu_print_taskHandle;
osThreadId cmd_execution_tHandle;
osThreadId uart_write_taskHandle;
osMessageQId cmd_queueHandle;
osMessageQId msg_queueHandle;
/* USER CODE BEGIN PV */

uint8_t cmd_recv_status = 0;
const char menu[] =
		"\nComandos Disponiveis: \n  LED_ON\n  LED_OFF\n  LED_TOGGLE_START\n  LED_STATUS_READ\n  RTC_DATETIME_READ\nDigite o comando a seguir: \n";
typedef enum Task {
	IDLE, PISCA, LIGA, DESLIGA, LEITURA, TEMPO
} enTask;

typedef struct APP_CMD {
	enTask Comando;
	char cmd[20];
} App_Cmd_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartCmdHandling(void const * argument);
void StartMenuPrint(void const * argument);
void StartCmdExecution(void const * argument);
void StartUartWrite(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of cmd_queue */
  osMessageQDef(cmd_queue, 16, App_Cmd_t*);
  cmd_queueHandle = osMessageCreate(osMessageQ(cmd_queue), NULL);

  /* definition and creation of msg_queue */
  osMessageQDef(msg_queue, 16, char*);
  msg_queueHandle = osMessageCreate(osMessageQ(msg_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of cmd_handling_ta */
  osThreadDef(cmd_handling_ta, StartCmdHandling, osPriorityNormal, 0, 128);
  cmd_handling_taHandle = osThreadCreate(osThread(cmd_handling_ta), NULL);

  /* definition and creation of menu_print_task */
  osThreadDef(menu_print_task, StartMenuPrint, osPriorityLow, 0, 128);
  menu_print_taskHandle = osThreadCreate(osThread(menu_print_task), NULL);

  /* definition and creation of cmd_execution_t */
  osThreadDef(cmd_execution_t, StartCmdExecution, osPriorityNormal, 0, 128);
  cmd_execution_tHandle = osThreadCreate(osThread(cmd_execution_t), NULL);

  /* definition and creation of uart_write_task */
  osThreadDef(uart_write_task, StartUartWrite, osPriorityNormal, 0, 128);
  uart_write_taskHandle = osThreadCreate(osThread(uart_write_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance->DR != '\n') {
		// Lê o bit recebido
		//*huart->pRxBuffPtr = huart->Instance->DR;

		// Incrementa o ponteiro
		//huart->pRxBuffPtr++;

		// Prepara o próximo recebimento
		HAL_UART_Receive_IT(&huart2, huart->pRxBuffPtr, 1);
	} else {
		// Lê o último bit recebido (p/ strlen)
		//*huart->pRxBuffPtr = huart->Instance->DR;

		// Ativa a flag de recv_status (recebido)
		//cmd_recv_status = SET;
		osSignalSet(cmd_handling_taHandle, 0x0001);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCmdHandling */
/**
 * @brief  Function implementing the cmd_handling_ta thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCmdHandling */
void StartCmdHandling(void const * argument)
{
  /* USER CODE BEGIN 5 */
	char cmd[20] = "Vai curinthia\n";
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), portMAX_DELAY);
	memset(cmd, 0, sizeof(cmd));
	HAL_UART_Receive_IT(&huart2, (uint8_t*) cmd, 1);
	osEvent evt;

	/* Infinite loop */
	for (;;) {
		evt = osSignalWait(0x0001, osWaitForever);
		if (evt.status == osEventSignal) {
			//HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), portMAX_DELAY);
			osMessagePut(msg_queueHandle, (uint32_t) &cmd, portMAX_DELAY);

			if (strcmp(cmd, "LED_ON\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0001);
			} else if (strcmp(cmd, "LED_OFF\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0002);
			} else if (strcmp(cmd, "LED_TOGGLE_START\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0003);
			} else if (strcmp(cmd, "LED_TOGGLE_STOP\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0004);
			} else if (strcmp(cmd, "LED_STATUS_READ\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0005);
			} else if (strcmp(cmd, "RTC_DATETIME_READ\n") == 0) {
				osSignalSet(cmd_execution_tHandle, 0x0006);
			}

			HAL_UART_Receive_IT(&huart2, (uint8_t*) cmd, 1);
		}
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartMenuPrint */
/**
 * @brief Function implementing the menu_print_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMenuPrint */
void StartMenuPrint(void const * argument)
{
  /* USER CODE BEGIN StartMenuPrint */

	/* Infinite loop */
	for (;;) {
		osMessagePut(msg_queueHandle, (uint32_t) &menu, portMAX_DELAY);
		evt = osSignalWait(0x0001, osWaitForever);
	}
  /* USER CODE END StartMenuPrint */
}

/* USER CODE BEGIN Header_StartCmdExecution */
/**
 * @brief Function implementing the cmd_execution_t thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCmdExecution */
void StartCmdExecution(void const * argument)
{
  /* USER CODE BEGIN StartCmdExecution */
	osEvent evt;
	uint8_t toggle = 0;
	char msg[20] = {0};
	evt = osSignalWait(0, osWaitForever);
	/* Infinite loop */
	for (;;) {

		if (evt.status == osEventSignal) {
			switch (evt.value.signals) {
			case 0x0001:
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
				evt = osSignalWait(0, osWaitForever);
				toggle = 0;
				break;
			case 0x0002:
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
				evt = osSignalWait(0, osWaitForever);
				toggle = 0;
				break;
			case 0x0003:
				//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				toggle = 1;
				break;
			case 0x0004:
				toggle = 0;
				evt = osSignalWait(0, osWaitForever);
				break;
			case 0x0005:
				memset(msg, 0, strlen(msg));
				if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin)){
					sprintf(msg, "HIGH\n");
				}else{
					sprintf(msg, "LOW\n");
				}
				osMessagePut(msg_queueHandle, (uint32_t) &msg, portMAX_DELAY);
				evt = osSignalWait(0, 0);
				break;
			default:
				evt = osSignalWait(0, osWaitForever);

			}
		}
		if(toggle){
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			evt = osSignalWait(0, 1000);
		}
	}
  /* USER CODE END StartCmdExecution */
}

/* USER CODE BEGIN Header_StartUartWrite */
/**
 * @brief Function implementing the uart_write_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartWrite */
void StartUartWrite(void const * argument)
{
  /* USER CODE BEGIN StartUartWrite */
	osEvent evt;
	char *pmsg = NULL;
	/* Infinite loop */
	for (;;) {
		evt = osMessageGet(msg_queueHandle, portMAX_DELAY);
		pmsg = evt.value.p;
		HAL_UART_Transmit(&huart2, (uint8_t*) pmsg, strlen(pmsg),
				portMAX_DELAY);
		memset(pmsg, 0, strlen(pmsg));
	}
  /* USER CODE END StartUartWrite */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
