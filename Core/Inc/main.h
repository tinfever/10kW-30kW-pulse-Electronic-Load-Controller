/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TIM_HandleTypeDef htim11;	// Fan PWM
extern TIM_HandleTypeDef htim3;		// Rotary Encoder

//extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FET_EN42_Pin GPIO_PIN_2
#define FET_EN42_GPIO_Port GPIOE
#define FET_EN41_Pin GPIO_PIN_3
#define FET_EN41_GPIO_Port GPIOE
#define FET_EN40_Pin GPIO_PIN_4
#define FET_EN40_GPIO_Port GPIOE
#define FET_EN39_Pin GPIO_PIN_5
#define FET_EN39_GPIO_Port GPIOE
#define FET_EN38_Pin GPIO_PIN_6
#define FET_EN38_GPIO_Port GPIOE
#define FET_EN37_Pin GPIO_PIN_9
#define FET_EN37_GPIO_Port GPIOI
#define FET_EN36_Pin GPIO_PIN_10
#define FET_EN36_GPIO_Port GPIOI
#define FET_EN35_Pin GPIO_PIN_11
#define FET_EN35_GPIO_Port GPIOI
#define FET_EN34_Pin GPIO_PIN_12
#define FET_EN34_GPIO_Port GPIOI
#define FET_EN33_Pin GPIO_PIN_13
#define FET_EN33_GPIO_Port GPIOI
#define FET_EN32_Pin GPIO_PIN_14
#define FET_EN32_GPIO_Port GPIOI
#define IMUX_OUT11_Pin GPIO_PIN_3
#define IMUX_OUT11_GPIO_Port GPIOF
#define IMUX_OUT10_Pin GPIO_PIN_4
#define IMUX_OUT10_GPIO_Port GPIOF
#define IMUX_OUT12_Pin GPIO_PIN_5
#define IMUX_OUT12_GPIO_Port GPIOF
#define IMUX_OUT13_Pin GPIO_PIN_6
#define IMUX_OUT13_GPIO_Port GPIOF
#define IMUX_OUT14_Pin GPIO_PIN_7
#define IMUX_OUT14_GPIO_Port GPIOF
#define IMUX_OUT15_Pin GPIO_PIN_0
#define IMUX_OUT15_GPIO_Port GPIOC
#define IMUX_OUT9_Pin GPIO_PIN_1
#define IMUX_OUT9_GPIO_Port GPIOC
#define IMUX_OUT8_Pin GPIO_PIN_2
#define IMUX_OUT8_GPIO_Port GPIOC
#define IMUX_OUT6_Pin GPIO_PIN_3
#define IMUX_OUT6_GPIO_Port GPIOC
#define REMOTE_VSENSE_Pin GPIO_PIN_0
#define REMOTE_VSENSE_GPIO_Port GPIOA
#define I_SUM_Pin GPIO_PIN_1
#define I_SUM_GPIO_Port GPIOA
#define IMUX_OUT7_Pin GPIO_PIN_2
#define IMUX_OUT7_GPIO_Port GPIOA
#define IMUX_OUT4_Pin GPIO_PIN_3
#define IMUX_OUT4_GPIO_Port GPIOA
#define TC_MUX_Pin GPIO_PIN_4
#define TC_MUX_GPIO_Port GPIOA
#define THERMISTOR_MUX_Pin GPIO_PIN_5
#define THERMISTOR_MUX_GPIO_Port GPIOA
#define IMUX_OUT1_Pin GPIO_PIN_6
#define IMUX_OUT1_GPIO_Port GPIOA
#define IMUX_OUT0_Pin GPIO_PIN_7
#define IMUX_OUT0_GPIO_Port GPIOA
#define IMUX_OUT5_Pin GPIO_PIN_4
#define IMUX_OUT5_GPIO_Port GPIOC
#define IMUX_OUT2_Pin GPIO_PIN_0
#define IMUX_OUT2_GPIO_Port GPIOB
#define IMUX_OUT3_Pin GPIO_PIN_1
#define IMUX_OUT3_GPIO_Port GPIOB
#define FET_EN7_Pin GPIO_PIN_15
#define FET_EN7_GPIO_Port GPIOI
#define TMUX_INH0_Pin GPIO_PIN_0
#define TMUX_INH0_GPIO_Port GPIOJ
#define TMUX_INH1_Pin GPIO_PIN_1
#define TMUX_INH1_GPIO_Port GPIOJ
#define TMUX_INH2_Pin GPIO_PIN_2
#define TMUX_INH2_GPIO_Port GPIOJ
#define TMUX_INH3_Pin GPIO_PIN_3
#define TMUX_INH3_GPIO_Port GPIOJ
#define TMUX_INH4_Pin GPIO_PIN_4
#define TMUX_INH4_GPIO_Port GPIOJ
#define FET_EN6_Pin GPIO_PIN_0
#define FET_EN6_GPIO_Port GPIOG
#define FET_EN5_Pin GPIO_PIN_1
#define FET_EN5_GPIO_Port GPIOG
#define FET_EN4_Pin GPIO_PIN_7
#define FET_EN4_GPIO_Port GPIOE
#define FET_EN3_Pin GPIO_PIN_8
#define FET_EN3_GPIO_Port GPIOE
#define FET_EN2_Pin GPIO_PIN_9
#define FET_EN2_GPIO_Port GPIOE
#define FET_EN1_Pin GPIO_PIN_10
#define FET_EN1_GPIO_Port GPIOE
#define FET_EN0_Pin GPIO_PIN_11
#define FET_EN0_GPIO_Port GPIOE
#define FET_EN15_Pin GPIO_PIN_12
#define FET_EN15_GPIO_Port GPIOE
#define FET_EN14_Pin GPIO_PIN_13
#define FET_EN14_GPIO_Port GPIOE
#define FET_EN13_Pin GPIO_PIN_14
#define FET_EN13_GPIO_Port GPIOE
#define FET_EN12_Pin GPIO_PIN_15
#define FET_EN12_GPIO_Port GPIOE
#define TFT_SCLK_Pin GPIO_PIN_10
#define TFT_SCLK_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_11
#define IO1_GPIO_Port GPIOB
#define TMUX_INH5_Pin GPIO_PIN_5
#define TMUX_INH5_GPIO_Port GPIOJ
#define TMUX_S0_Pin GPIO_PIN_11
#define TMUX_S0_GPIO_Port GPIOH
#define TMUX_S1_Pin GPIO_PIN_12
#define TMUX_S1_GPIO_Port GPIOH
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_13
#define IO2_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_14
#define SD_MISO_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define FET_EN11_Pin GPIO_PIN_8
#define FET_EN11_GPIO_Port GPIOD
#define FET_EN10_Pin GPIO_PIN_9
#define FET_EN10_GPIO_Port GPIOD
#define FET_EN9_Pin GPIO_PIN_10
#define FET_EN9_GPIO_Port GPIOD
#define FET_EN8_Pin GPIO_PIN_11
#define FET_EN8_GPIO_Port GPIOD
#define FET_EN23_Pin GPIO_PIN_12
#define FET_EN23_GPIO_Port GPIOD
#define FET_EN22_Pin GPIO_PIN_13
#define FET_EN22_GPIO_Port GPIOD
#define FET_EN21_Pin GPIO_PIN_14
#define FET_EN21_GPIO_Port GPIOD
#define FET_EN20_Pin GPIO_PIN_15
#define FET_EN20_GPIO_Port GPIOD
#define TMUX_INH6_Pin GPIO_PIN_6
#define TMUX_INH6_GPIO_Port GPIOJ
#define TMUX_INH7_Pin GPIO_PIN_7
#define TMUX_INH7_GPIO_Port GPIOJ
#define TMUX_INH8_Pin GPIO_PIN_8
#define TMUX_INH8_GPIO_Port GPIOJ
#define TMUX_INH9_Pin GPIO_PIN_9
#define TMUX_INH9_GPIO_Port GPIOJ
#define TMUX_INH10_Pin GPIO_PIN_10
#define TMUX_INH10_GPIO_Port GPIOJ
#define TMUX_INH11_Pin GPIO_PIN_11
#define TMUX_INH11_GPIO_Port GPIOJ
#define FET_EN19_Pin GPIO_PIN_2
#define FET_EN19_GPIO_Port GPIOG
#define FET_EN18_Pin GPIO_PIN_3
#define FET_EN18_GPIO_Port GPIOG
#define FET_EN17_Pin GPIO_PIN_4
#define FET_EN17_GPIO_Port GPIOG
#define FET_EN16_Pin GPIO_PIN_5
#define FET_EN16_GPIO_Port GPIOG
#define FET_EN31_Pin GPIO_PIN_6
#define FET_EN31_GPIO_Port GPIOG
#define FET_EN30_Pin GPIO_PIN_7
#define FET_EN30_GPIO_Port GPIOG
#define FET_EN29_Pin GPIO_PIN_8
#define FET_EN29_GPIO_Port GPIOG
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOC
#define IMUX_S0_Pin GPIO_PIN_13
#define IMUX_S0_GPIO_Port GPIOH
#define IMUX_S1_Pin GPIO_PIN_14
#define IMUX_S1_GPIO_Port GPIOH
#define FET_EN28_Pin GPIO_PIN_0
#define FET_EN28_GPIO_Port GPIOI
#define FET_EN27_Pin GPIO_PIN_1
#define FET_EN27_GPIO_Port GPIOI
#define FET_EN26_Pin GPIO_PIN_2
#define FET_EN26_GPIO_Port GPIOI
#define FET_EN25_Pin GPIO_PIN_3
#define FET_EN25_GPIO_Port GPIOI
#define SW3_Pin GPIO_PIN_10
#define SW3_GPIO_Port GPIOC
#define ENC_SW_Pin GPIO_PIN_11
#define ENC_SW_GPIO_Port GPIOC
#define REVERSE_VOLTAGE_ALERT_Pin GPIO_PIN_12
#define REVERSE_VOLTAGE_ALERT_GPIO_Port GPIOC
#define FET_EN24_Pin GPIO_PIN_0
#define FET_EN24_GPIO_Port GPIOD
#define FET_EN63_Pin GPIO_PIN_1
#define FET_EN63_GPIO_Port GPIOD
#define FET_EN62_Pin GPIO_PIN_2
#define FET_EN62_GPIO_Port GPIOD
#define FET_EN61_Pin GPIO_PIN_3
#define FET_EN61_GPIO_Port GPIOD
#define FET_EN60_Pin GPIO_PIN_4
#define FET_EN60_GPIO_Port GPIOD
#define FET_EN59_Pin GPIO_PIN_5
#define FET_EN59_GPIO_Port GPIOD
#define FET_EN58_Pin GPIO_PIN_6
#define FET_EN58_GPIO_Port GPIOD
#define FET_EN57_Pin GPIO_PIN_7
#define FET_EN57_GPIO_Port GPIOD
#define TMUX_INH12_Pin GPIO_PIN_12
#define TMUX_INH12_GPIO_Port GPIOJ
#define TMUX_INH13_Pin GPIO_PIN_13
#define TMUX_INH13_GPIO_Port GPIOJ
#define TMUX_INH14_Pin GPIO_PIN_14
#define TMUX_INH14_GPIO_Port GPIOJ
#define TMUX_INH15_Pin GPIO_PIN_15
#define TMUX_INH15_GPIO_Port GPIOJ
#define FET_EN56_Pin GPIO_PIN_9
#define FET_EN56_GPIO_Port GPIOG
#define FET_EN55_Pin GPIO_PIN_10
#define FET_EN55_GPIO_Port GPIOG
#define FET_EN54_Pin GPIO_PIN_11
#define FET_EN54_GPIO_Port GPIOG
#define FET_EN53_Pin GPIO_PIN_12
#define FET_EN53_GPIO_Port GPIOG
#define FET_EN52_Pin GPIO_PIN_13
#define FET_EN52_GPIO_Port GPIOG
#define FET_EN51_Pin GPIO_PIN_14
#define FET_EN51_GPIO_Port GPIOG
#define FET_EN50_Pin GPIO_PIN_7
#define FET_EN50_GPIO_Port GPIOK
#define FET_EN49_Pin GPIO_PIN_15
#define FET_EN49_GPIO_Port GPIOG
#define TFT_RESET_Pin GPIO_PIN_4
#define TFT_RESET_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_5
#define TFT_DC_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_6
#define SD_CS_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_9
#define FAN_PWM_GPIO_Port GPIOB
#define FET_EN48_Pin GPIO_PIN_0
#define FET_EN48_GPIO_Port GPIOE
#define FET_EN47_Pin GPIO_PIN_1
#define FET_EN47_GPIO_Port GPIOE
#define FET_EN46_Pin GPIO_PIN_4
#define FET_EN46_GPIO_Port GPIOI
#define FET_EN45_Pin GPIO_PIN_5
#define FET_EN45_GPIO_Port GPIOI
#define FET_EN44_Pin GPIO_PIN_6
#define FET_EN44_GPIO_Port GPIOI
#define FET_EN43_Pin GPIO_PIN_7
#define FET_EN43_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

//#define ADC_ISENSE ADC_CHANNEL_0
//#define ADC_VSENSE ADC_CHANNEL_1
//#define ADC_IMUX_0B ADC_CHANNEL_12
//#define ADC_IMUX_0A ADC_CHANNEL_13
//#define ADC_TC_MUX ADC_CHANNEL_10
//#define ADC_THERM_MUX ADC_CHANNEL_11

// asterisk* = channel number overlap
// take care to only use macro for ADC channel number on correct ADC.
// ADC_CHANNEL_14 means a different pin on different ADCs

#define ADC_REMOTE_VSENSE ADC_CHANNEL_0		// ADC123
#define ADC_ISUM ADC_CHANNEL_1				// ADC123
#define ADC_TC_MUX ADC_CHANNEL_4						// ADC12 *
#define ADC_THERMISTOR_MUX ADC_CHANNEL_5				// ADC12 *

#define ADC_IMUX_OUT0 ADC_CHANNEL_7						// ADC12
#define ADC_IMUX_OUT1 ADC_CHANNEL_6						// ADC12
#define ADC_IMUX_OUT2 ADC_CHANNEL_8						// ADC12
#define ADC_IMUX_OUT3 ADC_CHANNEL_9						// ADC12 *
#define ADC_IMUX_OUT4 ADC_CHANNEL_3			// ADC123
#define ADC_IMUX_OUT5 ADC_CHANNEL_14					// ADC12 *
#define ADC_IMUX_OUT6 ADC_CHANNEL_13		// ADC123
#define ADC_IMUX_OUT7 ADC_CHANNEL_2			// ADC123
#define ADC_IMUX_OUT8 ADC_CHANNEL_12		// ADC123
#define ADC_IMUX_OUT9 ADC_CHANNEL_11		// ADC123
#define ADC_IMUX_OUT10 ADC_CHANNEL_14								// ADC3 *
#define ADC_IMUX_OUT11 ADC_CHANNEL_9								// ADC3 *
#define ADC_IMUX_OUT12 ADC_CHANNEL_15								// ADC3
#define ADC_IMUX_OUT13 ADC_CHANNEL_4								// ADC3 *
#define ADC_IMUX_OUT14 ADC_CHANNEL_5								// ADC3 *
#define ADC_IMUX_OUT15 ADC_CHANNEL_10		// ADC123

#define FAN_PWM_TIM htim11
#define ROTARY_ENCODER_TIM htim3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
