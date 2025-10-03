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

const uint16_t MODBUS_TIMEOUT = 50;

const uint8_t ULTRASONIC_ADDR = 0x01;
const uint16_t ULTRASONIC_REG_START_ADDR = 0x0000;
const uint16_t ULTRASONIC_BUFF_SIZE = 3;

const uint8_t CLIP_ADDR = 0x07;
const uint16_t CLIP_STATE_REG_START_ADDR = 0x0040;
const uint16_t CLIP_STATE_BUFF_SIZE = 2;
const uint16_t CLIP_FB_REG_START_ADDR = 0x0042;
const uint16_t CLIP_FB_BUFF_SIZE = 3;

const uint8_t VACUUM_ADDR = 0x12;
const uint16_t VACUUM_REG_START_ADDR = 0x0001;
const uint16_t VACUUM_BUFF_SIZE = 1;

const uint16_t VACUUM_REG_CONFIG_ADDR = 0x0013;
const uint16_t VACUUM_CONFIG_BUFF_SIZE = 3;

const uint16_t VACUUM_REG_RESET_ADDR = 0x0020;

const uint16_t VACUUM_CONFIG_PERIOD = 5; // 5 seconds
const uint16_t WATCHDOG_PER_SEC = 2;

CANopenNodeSTM32 canOpenNodeSTM32;

uint16_t ultrasonic_buff[3] = { 0, 0, 0 };

uint16_t clip_state_buff[2] = { 0, 0 };
uint32_t clip_fb_buff[3] = { 0, 0 };

bool clip_init_diff = false;
bool clip_position_diff = false;
bool clip_speed_diff = false;
bool clip_current_diff = false;
bool clip_point_diff = false;

uint8_t prev_clip_init = 0;
uint32_t prev_clip_position = 0;
uint32_t prev_clip_speed = 0;
uint32_t prev_clip_current = 0;
uint32_t prev_clip_point = 0;

uint8_t prev_ultrasonic_enable = 0;
uint8_t prev_clip_enable = 0;

uint16_t vacuum_buff[1] = { 0 };

uint16_t vacuum_config_buff[3] = { 0, 0, 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void canopen_process(void);

void ultrasonic_data_process(void);

void clip_data_process(void);
void clip_control_process(void);

void write_clip_u16(uint16_t od_addr, uint16_t modbus_addr);
void write_clip_u32(uint16_t od_addr, uint16_t modbus_addr);

void clip_u8_diff_validation(uint16_t od_addr, bool* diff, uint8_t* prev);
void clip_u32_diff_validation(uint16_t od_addr, bool* diff, uint32_t* prev);

void vacuum_data_process(void);
void vacuum_config_process(void);

void pump_ctrl(uint16_t index, GPIO_TypeDef *port, uint16_t pin);
void pump_state(uint16_t index, GPIO_TypeDef *port, uint16_t pin);

bool enable_validation(uint16_t od_addr);

void show_err_led(void);

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

	for (uint8_t i = 0; i < 14; i++)
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
	mmodbus_set32bitOrder(MModBus_32bitOrder_CDAB);

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

		if (enable_validation(0x603F))
		{
			clip_data_process();
			clip_control_process();
		}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
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

/* USER CODE BEGIN 4 */

void canopen_process(void)
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, !canOpenNodeSTM32.outStatusLEDGreen);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !canOpenNodeSTM32.outStatusLEDRed);

	canopen_app_process();
}

void ultrasonic_data_process(void)
{
	uint8_t ultrasonic_enable = 0;
	if (OD_get_u8(OD_find(OD, 0x601F), 0x00, &ultrasonic_enable, false) != ODR_OK)
	{
		show_err_led();
	}

	if (prev_ultrasonic_enable && !ultrasonic_enable)
	{
		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			ultrasonic_buff[i] = 0x0;
		}

		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			if (OD_set_u16(OD_find(OD, 0x6010 + i), 0x00, ultrasonic_buff[i], false) != ODR_OK)
				show_err_led();
		}
	}

	prev_ultrasonic_enable = ultrasonic_enable;

	if (!ultrasonic_enable)
		return;

	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readInputRegisters16i(ULTRASONIC_ADDR, ULTRASONIC_REG_START_ADDR, ULTRASONIC_BUFF_SIZE, (uint16_t*) &ultrasonic_buff))
	{
		for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
		{
			ultrasonic_buff[i] = 0xFFFF;
		}
		show_err_led();
	}

	for (uint8_t i = 0; i < ULTRASONIC_BUFF_SIZE; i++)
	{
		if (OD_set_u16(OD_find(OD, 0x6010 + i), 0x00, ultrasonic_buff[i], false) != ODR_OK)
			show_err_led();
	}

	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
}

void clip_data_process(void)
{
	HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);

	HAL_Delay(5); // FIXME: error may occur if without this delay

	if (!mmodbus_readHoldingRegisters16i(CLIP_ADDR, CLIP_STATE_REG_START_ADDR, CLIP_STATE_BUFF_SIZE, (uint16_t*) &clip_state_buff))
	{
		for (uint8_t i = 0; i < 2; i++)
		{
			clip_state_buff[i] = 0xFFFF;
		}
		show_err_led();
	}

	if (!mmodbus_readHoldingRegisters32i(CLIP_ADDR, CLIP_FB_REG_START_ADDR, CLIP_FB_BUFF_SIZE, (uint32_t*) &clip_fb_buff))
	{
		for (uint8_t i = 0; i < CLIP_FB_BUFF_SIZE; i++)
		{
			clip_fb_buff[i] = 0xFFFFFFFF;
		}
		show_err_led();
	}

	HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_SET);
}

void clip_control_process(void)
{
	HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_RESET);
	if (clip_init_diff)
	{
		write_clip_u16(0x6030, 0x0000);
		clip_init_diff = false;
	}

	if (clip_position_diff)
	{
		write_clip_u32(0x6031, 0x0002);
		clip_position_diff = false;
	}

	if (clip_speed_diff)
	{
		write_clip_u32(0x6032, 0x0004);
		clip_speed_diff = false;
	}

	if (clip_current_diff)
	{
		write_clip_u32(0x6033, 0x0006);
		clip_current_diff = false;
	}

	if (clip_point_diff)
	{
		write_clip_u32(0x6034, 0x0017);
		clip_point_diff = false;
	}

	HAL_GPIO_WritePin(LED_6_GPIO_Port, LED_6_Pin, GPIO_PIN_SET);
}

void write_clip_u16(uint16_t od_addr, uint16_t modbus_addr)
{
	uint8_t data_u8 = 0;
	if (OD_get_u8(OD_find(OD, od_addr), 0x00, &data_u8, false) != ODR_OK)
	{
		show_err_led();
	}

	uint16_t data_u16 = data_u8;

	HAL_Delay(5); // FIXME: error may occur if without this delay

	if (!mmodbus_writeHoldingRegisters16i(CLIP_ADDR, modbus_addr, 1, (uint16_t*) &data_u16))
	{
		show_err_led();
	}
}

void write_clip_u32(uint16_t od_addr, uint16_t modbus_addr)
{
	uint32_t data = 0;
	if (OD_get_u32(OD_find(OD, od_addr), 0x00, &data, false) != ODR_OK)
	{
		show_err_led();
	}

	data = (data >> 16) | (data << 16);

	HAL_Delay(5); // FIXME: error occur if without this delay

	if (!mmodbus_writeHoldingRegisters16i(CLIP_ADDR, modbus_addr, 2, (uint16_t*) &data))
	{
		show_err_led();
	}
}

void vacuum_data_process(void)
{
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_START_ADDR, VACUUM_BUFF_SIZE, (uint16_t*) &vacuum_buff))
	{
		for (uint8_t i = 0; i < VACUUM_BUFF_SIZE; i++)
		{
			vacuum_buff[i] = 0xFFFF;
		}
		show_err_led();
	}

	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
}

void vacuum_config_process(void)
{
	uint32_t running;
	if (OD_get_u32(OD_find(OD, RUNNING_ADDR), 0x00, &running, false) != ODR_OK)
	{
		show_err_led();
	}
	if (running % (VACUUM_CONFIG_PERIOD * WATCHDOG_PER_SEC) != 0)
		return;

	HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);

	if (!mmodbus_readHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_CONFIG_ADDR, VACUUM_CONFIG_BUFF_SIZE, (uint16_t*) &vacuum_config_buff))
	{
		show_err_led();
	}

	for (uint8_t i = 0; i < VACUUM_CONFIG_BUFF_SIZE; i++)
	{
		uint8_t data = 0;
		if (OD_get_u8(OD_find(OD, 0x6008 + i), 0x00, &data, false) != ODR_OK)
		{
			show_err_led();
			continue;
		}
		if (data == vacuum_config_buff[i])
		{
			continue;
		}

		uint16_t large_data = data;

		if (!mmodbus_writeHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_CONFIG_ADDR + i, 1, (uint16_t*) &large_data))
		{
			show_err_led();
		}
	}

	uint8_t reset = 0;
	if (OD_get_u8(OD_find(OD, 0x600B), 0x00, &reset, false) != ODR_OK)
	{
		show_err_led();
	}

	if (reset)
	{
		const uint8_t VACUUM_RESET_BUFF_SIZE = 1;
		uint16_t vacuum_reset_buff[1] = { 1 };

		if (!mmodbus_writeHoldingRegisters16i(VACUUM_ADDR, VACUUM_REG_RESET_ADDR, VACUUM_RESET_BUFF_SIZE, (uint16_t*) &vacuum_reset_buff))
		{
			show_err_led();
		}

		uint8_t clear = 0;
		if (OD_set_u8(OD_find(OD, 0x600A), 0x00, (uint8_t) clear, false) != ODR_OK)
		{
			show_err_led();
		}
	}

	HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);
}

void pump_ctrl(uint16_t index, GPIO_TypeDef *port, uint16_t pin)
{
	uint8_t ctrl = 0;

	if (OD_get_u8(OD_find(OD, index), 0x00, &ctrl, false) != ODR_OK)
	{
		show_err_led();
	}

	switch (index)
	{
	case 0x6020:
		HAL_GPIO_WritePin(port, pin, ctrl == 0x0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		break;
	case 0x6021:
		HAL_GPIO_WritePin(port, pin, ctrl == 0x0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	}
}

void pump_state(uint16_t index, GPIO_TypeDef *port, uint16_t pin)
{
	GPIO_PinState state = !HAL_GPIO_ReadPin(port, pin);

	switch (index)
	{
	case 0x6024:
		state = !state;
		break;
	case 0x6025:
		break;
	}

	if (OD_set_u8(OD_find(OD, index), 0x00, (uint8_t) state, false) != ODR_OK)
	{
		show_err_led();
	}
}

void show_err_led(void)
{
	HAL_GPIO_WritePin(LED_Default_GPIO_Port, LED_Default_Pin, GPIO_PIN_RESET);
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
//	HAL_GPIO_TogglePin(LED_Default_GPIO_Port, LED_Default_Pin);
	HAL_IWDG_Refresh(&hiwdg);

	uint32_t val;
	if (OD_get_u32(OD_find(OD, RUNNING_ADDR), 0x00, &val, false) != ODR_OK)
	{
		show_err_led();
	}

	val++;
	if (OD_set_u32(OD_find(OD, RUNNING_ADDR), 0x00, val, false) != ODR_OK)
	{
		show_err_led();
	}
}

void htim2_cb(void)
{
	uint8_t restart = 0;

	if (OD_get_u8(OD_find(OD, 0x6110), 0x00, &restart, false) != ODR_OK)
	{
		show_err_led();
	}

	if (restart)
	{
		while (1)
		{
			// trigger watchdog to restart MCU
			show_err_led();
			HAL_Delay(1000);
		}
	}
}

void htim3_cb(void)
{
	if (OD_set_i16(OD_find(OD, 0x6000), 0x00, (int16_t) vacuum_buff[0], false) != ODR_OK)
		show_err_led();

	pump_ctrl(0x6020, Pump_Power_GPIO_Port, Pump_Power_Pin);
	pump_ctrl(0x6021, Pump_Release_GPIO_Port, Pump_Release_Pin);

	pump_state(0x6024, Pump_Power_GPIO_Port, Pump_Power_Pin);
	pump_state(0x6025, Pump_Release_GPIO_Port, Pump_Release_Pin);

	if (OD_set_u8(OD_find(OD, 0x6035), 0x00, (uint8_t) clip_state_buff[0], false) != ODR_OK)
		show_err_led();

	if (OD_set_u8(OD_find(OD, 0x6036), 0x00, (uint8_t) clip_state_buff[1], false) != ODR_OK)
		show_err_led();

	if (OD_set_u32(OD_find(OD, 0x6037), 0x00, clip_fb_buff[0], false) != ODR_OK)
		show_err_led();

	if (OD_set_u32(OD_find(OD, 0x6038), 0x00, clip_fb_buff[1], false) != ODR_OK)
		show_err_led();

	if (OD_set_u32(OD_find(OD, 0x6039), 0x00, clip_fb_buff[2], false) != ODR_OK)
		show_err_led();

	clip_u8_diff_validation(0x6030, &clip_init_diff, &prev_clip_init);
	clip_u32_diff_validation(0x6031, &clip_position_diff, &prev_clip_position);
	clip_u32_diff_validation(0x6032, &clip_speed_diff, &prev_clip_speed);
	clip_u32_diff_validation(0x6033, &clip_current_diff, &prev_clip_current);
	clip_u32_diff_validation(0x6034, &clip_point_diff, &prev_clip_point);
}

void clip_u8_diff_validation(uint16_t od_addr, bool* diff, uint8_t* prev)
{
	uint8_t val = 0;

	if (OD_get_u8(OD_find(OD, od_addr), 0x00, &val, false) != ODR_OK)
	{
		show_err_led();
	}

	if (val != *prev)
	{
		*diff = true;
	}

	*prev = val;
}

void clip_u32_diff_validation(uint16_t od_addr, bool* diff, uint32_t* prev)
{
	uint32_t val = 0;

	if (OD_get_u32(OD_find(OD, od_addr), 0x00, &val, false) != ODR_OK)
	{
		show_err_led();
	}

	if (val != *prev)
	{
		*diff = true;
	}

	*prev = val;
}

bool enable_validation(uint16_t od_addr)
{
	uint8_t enable = 0;

	if (OD_get_u8(OD_find(OD, od_addr), 0x00, &enable, false) != ODR_OK)
	{
		show_err_led();
	}

	if (prev_clip_enable && !enable)
	{
		for (uint8_t i = 0; i < CLIP_STATE_BUFF_SIZE; i++)
		{
			clip_state_buff[i] = 0x0;
		}
		for (uint8_t i = 0; i < CLIP_FB_BUFF_SIZE; i++)
		{
			clip_fb_buff[i] = 0x0;
		}

		for (uint8_t i = 0; i < CLIP_STATE_BUFF_SIZE; i++)
		{
			if (OD_set_u8(OD_find(OD, 0x6035 + i), 0x00, clip_state_buff[i], false) != ODR_OK)
				show_err_led();
		}

		for (uint8_t i = 0; i < CLIP_FB_BUFF_SIZE; i++)
		{
			if (OD_set_u32(OD_find(OD, 0x6037 + i), 0x00, clip_fb_buff[i], false) != ODR_OK)
				show_err_led();
		}
	}

	prev_clip_enable = enable;

	return enable ? true : false;
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
