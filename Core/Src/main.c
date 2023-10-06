/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>

#include "state.h"
#include "display.h"
#include "adc.h"
#include "load.h"
#include "temp.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Allows printf to print to SWO
int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
}

void IO1_blink(void){
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_GPIO_TogglePin(IO1_GPIO_Port, IO1_Pin);
}

void IO2_blink(void){
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	HAL_GPIO_TogglePin(IO2_GPIO_Port, IO2_Pin);
}

void Stage_Sequence_Test(void){

	for (int i = 0; i < NUM_STAGES; i++){
		const LoadStageConfiguration* stage = GetPointerToSingleStageConfig(i);
		HAL_GPIO_WritePin(stage->io_port, stage->io_pin, 1);
		HAL_Delay(100);
	}

	HAL_Delay(5000);

	for (int i = 0; i < NUM_STAGES; i++){
		const LoadStageConfiguration* stage = GetPointerToSingleStageConfig(i);
		HAL_GPIO_WritePin(stage->io_port, stage->io_pin, 0);
		HAL_Delay(100);
	}

	HAL_Delay(5000);
}

typedef struct {
	bool enabled;
	uint32_t period_ms;
	uint32_t last_run_time;
	void (*RunTask)(void);
} Task;

void TaskScheduler(Task *task_list, uint32_t number_of_tasks){
	for (int i = 0; i < number_of_tasks; i++){
		uint32_t current_time = HAL_GetTick();
		if (task_list[i].enabled == true
			&& (current_time - task_list[i].last_run_time) >= task_list[i].period_ms
			&& task_list[i].RunTask != 0	//make sure task function pointer isn't null
			&& task_list[i].period_ms != 0	//and period isn't null either
		){
			task_list[i].last_run_time = current_time;
			task_list[i].RunTask();
		}
	}
}

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
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Encoder_Start(&ROTARY_ENCODER_TIM, TIM_CHANNEL_ALL);

  ADCInit();


  //start PWM fan control
  //default fan speed is 30%
//  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  //start debug cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

//  //Stop TIM4 during break while debugging
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;


#define NUM_TASKS 10
const uint32_t number_of_tasks = NUM_TASKS;
Task task_list[NUM_TASKS] = {0};

//task_list[0].RunTask = IO1_blink;
//task_list[0].period_ms = 500;
//task_list[0].enabled = true;

DisplayInit();
task_list[1].RunTask = DisplayUpdate;
task_list[1].period_ms = 50;
task_list[1].enabled = true;


//task_list[2].RunTask = LoadControl;
//task_list[2].period_ms = 50;
//task_list[2].enabled = true;
//
task_list[3].RunTask = UpdateLoadStageTemps;
task_list[3].period_ms = 500;
task_list[3].enabled = true;
//
//task_list[4].RunTask = FanSpeedControl;
//task_list[4].period_ms = 500;
//task_list[4].enabled = true;

task_list[5].RunTask = IO1_blink;
task_list[5].period_ms = 500;
task_list[5].enabled = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  TaskScheduler(task_list, number_of_tasks);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FET_EN42_Pin|FET_EN41_Pin|FET_EN40_Pin|FET_EN39_Pin
                          |FET_EN38_Pin|FET_EN4_Pin|FET_EN3_Pin|FET_EN2_Pin
                          |FET_EN1_Pin|FET_EN0_Pin|FET_EN15_Pin|FET_EN14_Pin
                          |FET_EN13_Pin|FET_EN12_Pin|FET_EN48_Pin|FET_EN47_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, FET_EN37_Pin|FET_EN36_Pin|FET_EN35_Pin|FET_EN34_Pin
                          |FET_EN33_Pin|FET_EN32_Pin|FET_EN7_Pin|FET_EN28_Pin
                          |FET_EN27_Pin|FET_EN26_Pin|FET_EN25_Pin|FET_EN46_Pin
                          |FET_EN45_Pin|FET_EN44_Pin|FET_EN43_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, TMUX_INH0_Pin|TMUX_INH1_Pin|TMUX_INH2_Pin|TMUX_INH3_Pin
                          |TMUX_INH4_Pin|TMUX_INH5_Pin|TMUX_INH6_Pin|TMUX_INH7_Pin
                          |TMUX_INH8_Pin|TMUX_INH9_Pin|TMUX_INH10_Pin|TMUX_INH11_Pin
                          |TMUX_INH12_Pin|TMUX_INH13_Pin|TMUX_INH14_Pin|TMUX_INH15_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, FET_EN6_Pin|FET_EN5_Pin|FET_EN19_Pin|FET_EN18_Pin
                          |FET_EN17_Pin|FET_EN16_Pin|FET_EN31_Pin|FET_EN30_Pin
                          |FET_EN29_Pin|FET_EN56_Pin|FET_EN55_Pin|FET_EN54_Pin
                          |FET_EN53_Pin|FET_EN52_Pin|FET_EN51_Pin|FET_EN49_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IO1_Pin|IO2_Pin|TFT_RESET_Pin|TFT_DC_Pin
                          |SD_CS_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, TMUX_S0_Pin|TMUX_S1_Pin|IMUX_S0_Pin|IMUX_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FET_EN11_Pin|FET_EN10_Pin|FET_EN9_Pin|FET_EN8_Pin
                          |FET_EN23_Pin|FET_EN22_Pin|FET_EN21_Pin|FET_EN20_Pin
                          |FET_EN24_Pin|FET_EN63_Pin|FET_EN62_Pin|FET_EN61_Pin
                          |FET_EN60_Pin|FET_EN59_Pin|FET_EN58_Pin|FET_EN57_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FET_EN50_GPIO_Port, FET_EN50_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FET_EN42_Pin FET_EN41_Pin FET_EN40_Pin FET_EN39_Pin
                           FET_EN38_Pin FET_EN4_Pin FET_EN3_Pin FET_EN2_Pin
                           FET_EN1_Pin FET_EN0_Pin FET_EN15_Pin FET_EN14_Pin
                           FET_EN13_Pin FET_EN12_Pin FET_EN48_Pin FET_EN47_Pin */
  GPIO_InitStruct.Pin = FET_EN42_Pin|FET_EN41_Pin|FET_EN40_Pin|FET_EN39_Pin
                          |FET_EN38_Pin|FET_EN4_Pin|FET_EN3_Pin|FET_EN2_Pin
                          |FET_EN1_Pin|FET_EN0_Pin|FET_EN15_Pin|FET_EN14_Pin
                          |FET_EN13_Pin|FET_EN12_Pin|FET_EN48_Pin|FET_EN47_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FET_EN37_Pin FET_EN36_Pin FET_EN35_Pin FET_EN34_Pin
                           FET_EN33_Pin FET_EN32_Pin FET_EN7_Pin FET_EN28_Pin
                           FET_EN27_Pin FET_EN26_Pin FET_EN25_Pin FET_EN46_Pin
                           FET_EN45_Pin FET_EN44_Pin FET_EN43_Pin */
  GPIO_InitStruct.Pin = FET_EN37_Pin|FET_EN36_Pin|FET_EN35_Pin|FET_EN34_Pin
                          |FET_EN33_Pin|FET_EN32_Pin|FET_EN7_Pin|FET_EN28_Pin
                          |FET_EN27_Pin|FET_EN26_Pin|FET_EN25_Pin|FET_EN46_Pin
                          |FET_EN45_Pin|FET_EN44_Pin|FET_EN43_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : TMUX_INH0_Pin TMUX_INH1_Pin TMUX_INH2_Pin TMUX_INH3_Pin
                           TMUX_INH4_Pin TMUX_INH5_Pin TMUX_INH6_Pin TMUX_INH7_Pin
                           TMUX_INH8_Pin TMUX_INH9_Pin TMUX_INH10_Pin TMUX_INH11_Pin
                           TMUX_INH12_Pin TMUX_INH13_Pin TMUX_INH14_Pin TMUX_INH15_Pin */
  GPIO_InitStruct.Pin = TMUX_INH0_Pin|TMUX_INH1_Pin|TMUX_INH2_Pin|TMUX_INH3_Pin
                          |TMUX_INH4_Pin|TMUX_INH5_Pin|TMUX_INH6_Pin|TMUX_INH7_Pin
                          |TMUX_INH8_Pin|TMUX_INH9_Pin|TMUX_INH10_Pin|TMUX_INH11_Pin
                          |TMUX_INH12_Pin|TMUX_INH13_Pin|TMUX_INH14_Pin|TMUX_INH15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : FET_EN6_Pin FET_EN5_Pin FET_EN19_Pin FET_EN18_Pin
                           FET_EN17_Pin FET_EN16_Pin FET_EN31_Pin FET_EN30_Pin
                           FET_EN29_Pin FET_EN56_Pin FET_EN55_Pin FET_EN54_Pin
                           FET_EN53_Pin FET_EN52_Pin FET_EN51_Pin FET_EN49_Pin */
  GPIO_InitStruct.Pin = FET_EN6_Pin|FET_EN5_Pin|FET_EN19_Pin|FET_EN18_Pin
                          |FET_EN17_Pin|FET_EN16_Pin|FET_EN31_Pin|FET_EN30_Pin
                          |FET_EN29_Pin|FET_EN56_Pin|FET_EN55_Pin|FET_EN54_Pin
                          |FET_EN53_Pin|FET_EN52_Pin|FET_EN51_Pin|FET_EN49_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : IO1_Pin IO2_Pin TFT_RESET_Pin TFT_DC_Pin
                           SD_CS_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = IO1_Pin|IO2_Pin|TFT_RESET_Pin|TFT_DC_Pin
                          |SD_CS_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TMUX_S0_Pin TMUX_S1_Pin IMUX_S0_Pin IMUX_S1_Pin */
  GPIO_InitStruct.Pin = TMUX_S0_Pin|TMUX_S1_Pin|IMUX_S0_Pin|IMUX_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : FET_EN11_Pin FET_EN10_Pin FET_EN9_Pin FET_EN8_Pin
                           FET_EN23_Pin FET_EN22_Pin FET_EN21_Pin FET_EN20_Pin
                           FET_EN24_Pin FET_EN63_Pin FET_EN62_Pin FET_EN61_Pin
                           FET_EN60_Pin FET_EN59_Pin FET_EN58_Pin FET_EN57_Pin */
  GPIO_InitStruct.Pin = FET_EN11_Pin|FET_EN10_Pin|FET_EN9_Pin|FET_EN8_Pin
                          |FET_EN23_Pin|FET_EN22_Pin|FET_EN21_Pin|FET_EN20_Pin
                          |FET_EN24_Pin|FET_EN63_Pin|FET_EN62_Pin|FET_EN61_Pin
                          |FET_EN60_Pin|FET_EN59_Pin|FET_EN58_Pin|FET_EN57_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin ENC_SW_Pin
                           REVERSE_VOLTAGE_ALERT_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|ENC_SW_Pin
                          |REVERSE_VOLTAGE_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FET_EN50_Pin */
  GPIO_InitStruct.Pin = FET_EN50_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FET_EN50_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
