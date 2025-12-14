/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern int counter;
extern int counter_UART2;
extern int counter_UART3;
extern int counter_CAN_BMS;
extern int counter_Chg_Current_Adc_over_target;
extern volatile uint16_t AuxBattery0_Info[];
extern volatile uint16_t AuxBattery1_Info[];

enum State {
	bad = 0, 
	good = 1
};

enum Mode_Selection {
	low = 0, 
	high
};

enum flag {
	no = 0,
	yes = 1
};

enum Input_Power_Stage {
	BAT_MODE_HIGH = 6,
	BAT_MODE_LOW = 7,
	AC_CONTROL = 8,
	AC_NO_CONTROL = 9
};




// Monitoring tool
extern float VI;
extern float VO;
extern float PH1_C;
extern float PH2_C;
extern float TOT_C;
extern float TH1;
extern float TH2;
extern float TH3;
extern float TH4;
extern float TH5;
extern uint8_t EV1;
extern uint8_t EV2;
extern uint8_t EV3;
extern uint8_t EV4;
extern uint8_t EV5;
extern uint8_t EV6;
extern uint8_t EV7;
extern uint8_t EV8;
extern uint8_t EV9;
extern uint8_t EV10;
extern uint8_t EV11;
extern uint8_t EV12;
extern uint8_t EV13;
extern uint8_t EV14;
extern uint8_t EV15;

extern volatile char RxDataChar[1];
extern char RxDataString[8];
extern char HeaderString[];;
extern volatile uint16_t RxDataIndex;
extern volatile uint8_t receive_cmd;

extern char buffer_MonitoringTool_Normal[300];


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc4;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c3;
// extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEBUG 1

#define LOW 						GPIO_PIN_RESET
#define HIGH 						GPIO_PIN_SET

#define ADC_RESOLUTION 		4095  
#define V_REF 						3.3 
#define	R31								100.0 
#define R93								27.0
#define	R101							100.0 
#define R250							27.0
#define	R99								1000.0
#define R100							20.0
#define	R169							1000.0
#define R182							20.0
#define	R228							1000.0
#define R173							20.0

#define R168							10
#define R165							10
#define R163							10
#define R161							10
#define R87								10

#define T0 								298.15
#define B25_50						3380.0
#define B25_80						3428.0
#define B25_85						3434.0
#define B25_100						3455.0
#define R0								10


#define ROUND(value) ((value) >= 0 ? (int)((value) + 0.5f) : (int)((value) - 0.5f))
#define LED_CONTROL(GPIO_PIN_STATE)	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_STATE)
#define READ_MODE_SELECTION 				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)

#define TURNON_LED									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define TURNOFF_LED									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define TOGGLE_LED									HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
