/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CO_app_STM32.h"
#include "OD.h"

#include "mmodbus.h"

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

/* USER CODE BEGIN PV */

const uint8_t NODE_ID = 0x55;
const uint16_t RUNNING_ADDR = 0x6100;

CANopenNodeSTM32 canOpenNodeSTM32;

const uint16_t MODBUS_TIMEOUT = 100;

const uint8_t ULTRASONIC_ADDR = 0x01;
const uint16_t ULTRASONIC_REG_START_ADDR = 0x0000;
const uint16_t ULTRASONIC_BUFF_SIZE = 3;
uint16_t ultrasonic_buff[3] =
{ 0, 0, 0 };

uint8_t prev_ultrasonic_enable = 0;

const uint8_t VACUUM_ADDR = 0x12;
const uint16_t VACUUM_REG_START_ADDR = 0x0001;
const uint16_t VACUUM_BUFF_SIZE = 1;
uint16_t vacuum_buff[1] =
{ 0 };

const uint16_t VACUUM_REG_CONFIG_ADDR = 0x0013;
const uint16_t VACUUM_CONFIG_BUFF_SIZE = 3;
uint16_t vacuum_config_buff[3] =
{ 0, 0, 0 };

const uint16_t VACUUM_REG_RESET_ADDR = 0x0020;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void canopen_process(void);

void ultrasonic_data_process(void);

void vacuum_data_process(void);
void vacuum_config_process(void);

void valve_ctrl(uint16_t index, GPIO_TypeDef *port, uint16_t pin);
void valve_state(uint16_t index, GPIO_TypeDef *port, uint16_t pin);

void htim1_cb();
void htim2_cb();
void htim3_cb();
void htim4_cb();

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
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_CAN_Init();
	MX_IWDG_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim1);

	for (uint8_t i = 0; i < 15; i++)
	{
		HAL_GPIO_TogglePin(LED_Default_GPIO_Port, LED_Default_Pin);
		HAL_Delay(200);
	}

	canOpenNodeSTM32.CANHandle = &hcan;
	canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
	canOpenNodeSTM32.timerHandle = &htim4;
	canOpenNodeSTM32.desiredNodeID = NODE_ID;
	canOpenNodeSTM32.baudrate = 1000;
	canopen_app_init(&canOpenNodeSTM32);

	mmodbus_init(MODBUS_TIMEOUT);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		canopen_process();

		ultrasonic_data_process();

		vacuum_data_process();

		vacuum_config_process();

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void canopen_process(void)
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin,
			!canOpenNodeSTM32.outStatusLEDGreen);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin,
			!canOpenNodeSTM32.outStatusLEDRed);

	canopen_app_process();
}

void ultrasonic_data_process(void)
{
	uint8_t ultrasonic_enable = 0;
	if (OD_get_u8(OD_find(OD, 0x601F), 0x00, &ultrasonic_enable, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	if (prev_ultrasonic_enable && !ultrasonic_enable)
	{
		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			ultrasonic_buff[i] = 0x0;
		}

		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			if (OD_set_u16(OD_find(OD, 0x6010 + i), 0x00, ultrasonic_buff[i], false)
					!= ODR_OK)
				HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
		}
	}

	prev_ultrasonic_enable = ultrasonic_enable;

	if (!ultrasonic_enable)
		return;

	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readInputRegisters16i(ULTRASONIC_ADDR,
			ULTRASONIC_REG_START_ADDR, ULTRASONIC_BUFF_SIZE,
			(uint16_t*) &ultrasonic_buff))
	{
		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			ultrasonic_buff[i] = 0xFFFF;
		}
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
	{
		if (OD_set_u16(OD_find(OD, 0x6010 + i), 0x00, ultrasonic_buff[i], false)
				!= ODR_OK)
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void vacuum_data_process(void)
{
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_START_ADDR,
			VACUUM_BUFF_SIZE, (uint16_t*) &vacuum_buff))
	{
		for (uint8_t i = 0; i < VACUUM_BUFF_SIZE; i++)
		{
			vacuum_buff[i] = 0xFFFF;
		}
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}


	if (OD_set_i16(OD_find(OD, 0x6000), 0x00, (int16_t) vacuum_buff[0], false) != ODR_OK)
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
}

void vacuum_config_process(void)
{
	HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_CONFIG_ADDR, VACUUM_CONFIG_BUFF_SIZE, (uint16_t*) &vacuum_config_buff))
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	for (uint8_t i = 0; i < VACUUM_CONFIG_BUFF_SIZE; i++)
	{
		uint8_t data = 0;
		if (OD_get_u8(OD_find(OD, 0x6008 + i), 0x00, &data, false) != ODR_OK)
		{
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
			continue;
		}
		if (data == vacuum_config_buff[i])
		{
			continue;
		}

		uint16_t large_data = data;

		if (!mmodbus_writeHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_CONFIG_ADDR + i, 1, (uint16_t*) &large_data))
		{
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
		}
	}

	uint8_t reset = 0;
	if (OD_get_u8(OD_find(OD, 0x600B), 0x00, &reset, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	if (reset)
	{
		const uint8_t VACUUM_RESET_BUFF_SIZE = 1;
		uint16_t vacuum_reset_buff[1] =
		{ 1 };

		if (!mmodbus_writeHoldingRegisters16i(VACUUM_ADDR,
				VACUUM_REG_RESET_ADDR, VACUUM_RESET_BUFF_SIZE,
				(uint16_t*) &vacuum_reset_buff))
		{
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
		}

		uint8_t clear = 0;
		if (OD_set_u8(OD_find(OD, 0x600A), 0x00, (uint8_t) clear, false)
				!= ODR_OK)
		{
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
		}
	}

	HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);
}

void valve_ctrl(uint16_t index, GPIO_TypeDef *port, uint16_t pin)
{
	uint8_t ctrl = 0;

	if (OD_get_u8(OD_find(OD, index), 0x00, &ctrl, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(port, pin, ctrl == 0x0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void valve_state(uint16_t index, GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_PinState state = !HAL_GPIO_ReadPin(port, pin);

	if (OD_set_u8(OD_find(OD, index), 0x00, (uint8_t) state, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1))
	{
		htim1_cb();
	}
	else if (htim == (&htim2))
	{
		htim2_cb();
	}
	else if (htim == (&htim3))
	{
		htim3_cb();
	}
	else if (htim == canopenNodeSTM32->timerHandle)
	{
		htim4_cb();
	}
}

void htim1_cb(void)
{
	HAL_GPIO_TogglePin(LED_Default_GPIO_Port, LED_Default_Pin);
	HAL_IWDG_Refresh(&hiwdg);

	uint32_t val;
	if (OD_get_u32(OD_find(OD, RUNNING_ADDR), 0x00, &val, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	val++;
	if (OD_set_u32(OD_find(OD, RUNNING_ADDR), 0x00, val, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}
}

void htim2_cb(void)
{
	uint8_t restart = 0;

	if (OD_get_u8(OD_find(OD, 0x6110), 0x00, &restart, false) != ODR_OK)
	{
		HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	}

	if (restart)
	{
		while (1)
		{
			// trigger watchdog to restart MCU
			HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
			HAL_Delay(1000);
		}
	}
}

void htim3_cb(void)
{
	valve_ctrl(0x6020, Valve_1_GPIO_Port, Valve_1_Pin);
	valve_ctrl(0x6021, Valve_2_GPIO_Port, Valve_2_Pin);

	valve_state(0x6024, Valve_1_GPIO_Port, Valve_1_Pin);
	valve_state(0x6025, Valve_1_GPIO_Port, Valve_1_Pin);
}

void htim4_cb(void)
{
	canopen_app_interrupt();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
