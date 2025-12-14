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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "charger.h"
#include "process_data.h"
#include "fdcan.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// #define ROUND(value) ((value) >= 0 ? (int)((value) + 0.5f) : (int)((value) - 0.5f))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc4;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t enable_charger_controller = 0;
uint8_t flag_BMS0_read = 1;
uint8_t flag_BMS1_read = 1;
volatile uint8_t flag_BMS0_alive = 0, flag_BMS1_alive = 0;
uint32_t pre_time_BMS0, cur_time_BMS0;
uint32_t pre_time_BMS1, cur_time_BMS1;

char buffer_stage[40] = {0};
char buffer_case[40] = {0};
uint16_t u16_ADC1_Val[2] = {0};
uint16_t u16_ADC2_Val[9] = {0};
uint16_t u16_ADC4_Val[2] = {0};
uint16_t u16_ADC1_Val_average[2] = {0};
uint16_t u16_ADC2_Val_average[9] = {0};
uint16_t u16_ADC4_Val_average[2] = {0};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC4_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
  { 
	  uint8_t RxLength = 0;
		HAL_UART_Receive_IT(&huart3, (uint8_t*)RxDataChar, 1);
		
//		if (RxDataChar[0] == HeaderString[RxDataIndex] || RxDataIndex > 5) {		// Check data has header 'EBIKE-'
		if (RxDataChar[0] == HeaderString[RxDataIndex] || RxDataIndex > 2) {			// Check data has header 'EB-'
			RxDataString[RxDataIndex] = RxDataChar[0];
			RxDataIndex++;
			if(RxDataChar[0] == ';' || RxDataIndex == 5) { // current receive each line
				receive_cmd = 1;
				RxDataIndex = 0;
			}
		} else {
			memset(RxDataString, 0, sizeof(RxDataString));
			RxDataString[0] = RxDataChar[0];
			RxDataIndex = 1;
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
  SCB->VTOR = (uint32_t)0x08040000;
	__enable_irq();
	char buffer[] = "hello\r\n";
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC4_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // init Independent Watchdog Timeout
//	while (!enable_charger_controller);
  //HAL_IWDG_Init(&hiwdg);
	
	FDCAN_Config();
	HAL_UART_Receive_IT(&huart3, (uint8_t*)RxDataChar, 1);

	// Calibrate ADC before call HAL_ADC_Start/HAL_ADC_Start_DMA
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
	HAL_Delay(100);
	// Trigger read ADC DMA in contious mode
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)u16_ADC1_Val, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)u16_ADC2_Val, 9);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t*)u16_ADC4_Val, 2);
	HAL_Delay(100);
	
	// BUCK mode
	PID_Init(&pid_caseD1, 200, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD2, 250, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD3, 80, 2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD4, 80, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD4_Phase1_Cur, 80, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD4_Phase2_Cur, 80, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD5, 80, 21, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD5_Phase1_Cur, 80, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD5_Phase2_Cur, 80, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseD6, 80, 20, 0.05); // TSetting Kp, Ki, Kd. Need to be change
	
	// BOOST mode
	PID_Init(&pid_caseU11, 50, 1.1, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU12, 50, 3.5, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU12, 50, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU14, 60, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU14_Phase1_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU14_Phase2_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU15, 60, 0.01, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU15_Phase1_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU15_Phase2_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
	PID_Init(&pid_caseU16, 70, 0.01, 0.05); // TSetting Kp, Ki, Kd. Need to be change
  
	// enable Timer 2 Interupt
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	// Send version
	HAL_UART_Transmit(&huart3, (uint8_t *)"v1.1.1\r\n", strlen("v1.0.1\r\n"), HAL_MAX_DELAY);
#ifdef DEBUG
	memset(buffer_stage, 0, sizeof(buffer_stage));
	snprintf(buffer_stage, 40, "current_stage = %s\r\n", List_Stage[current_stage]);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer_stage, strlen(buffer_stage), HAL_MAX_DELAY);
#endif

	pre_time_BMS0 = HAL_GetTick();
	pre_time_BMS1 = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// DEBUG stage and case.
#ifdef DEBUG
		if (previous_stage != current_stage) {
			memset(buffer_stage, 0, sizeof(buffer_stage));
			snprintf(buffer_stage, 40, "current_stage = %s\r\n", List_Stage[current_stage]);
			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_stage, strlen(buffer_stage), HAL_MAX_DELAY);
			previous_stage = current_stage;
		}
#endif
		if (previous_case != current_case) {
			flag1_D6 = flag2_D6 = 0;
      flag1_U16 = flag2_U16 = 0;
			counter_Chg_Current_Adc_over_target = 0;
			// Init again PID everytime CASE is changed
			// BUCK mode
			PID_Init(&pid_caseD1, 200, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD2, 250, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD3, 80, 2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD4, 80, 20, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD4_Phase1_Cur, 80, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD4_Phase2_Cur, 80, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD5, 80, 21, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD5_Phase1_Cur, 80, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD5_Phase2_Cur, 80, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseD6, 80, 20, 0.05); // TSetting Kp, Ki, Kd. Need to be change
			// BOOST mode
			PID_Init(&pid_caseU11, 50, 1.1, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU12, 50, 3.5, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU12, 50, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU14, 60, 5, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU14_Phase1_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU14_Phase2_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU15, 60, 0.01, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU15_Phase1_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU15_Phase2_Cur, 8, 1.2, 0.05); // Setting Kp, Ki, Kd. Need to be change
			PID_Init(&pid_caseU16, 70, 0.01, 0.05); // TSetting Kp, Ki, Kd. Need to be change
	
			if (hardware_protection_flag == 0 && software_protection_flag == 0) {
				if (current_case == caseD1 || current_case == caseD3 || current_case == caseD6) {
					// Start BUCK1, Stop BUCK2 & BOOST1 & BOOST2
					HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);						// BUCK1
					HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);						// BUCK2
					HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
					
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);							// BOOST1
					HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);							// BOOST2
					HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
					
					duty_cycle_BUCK1 = 10;
					duty_cycle_BUCK2 = 0;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;
				} else if (current_case == caseD2) {
					// Start BUCK1, Stop BUCK2 & BOOST1 & BOOST2
					HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);						// BUCK1
					HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);						// BUCK2
					HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
					
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);							// BOOST1
					HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);							// BOOST2
					HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
					
					duty_cycle_BUCK1 = 0;
					duty_cycle_BUCK2 = 0;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;				
				} else if (current_case == caseD4 || current_case == caseD5) {
					// Start BUCK1 & BUCK2, Stop BOOST1 & BOOST2
//					HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);						// BUCK1
//					HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
//					HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);						// BUCK2
//					HAL_TIMEx_PWMN_Start(&htim17,TIM_CHANNEL_1);
					
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);							// BOOST1
					HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);							// BOOST2
					HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
					
					duty_cycle_BUCK1 = 10;
					duty_cycle_BUCK2 = 10;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;					
				} else if (current_case == caseU11 || current_case == caseU12 || current_case == caseU13 || current_case == caseU16) {
					// Start BOOST1, Stop BUCK1 & BUCK2 & BOOST2
					HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);						// BUCK1
					HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);						// BUCK2
					HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
					
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);						// BOOST1
					HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);							// BOOST2
					HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
					
					duty_cycle_BUCK1 = 0;
					duty_cycle_BUCK2 = 0;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;
				} else if (current_case == caseU14 || current_case == caseU15) {
					// Start BOOST1 & BOOST2, Stop BUCK1 & BUCK2
					HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);						// BUCK1
					HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);						// BUCK2
					HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
					
//					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);						// BOOST1
//					HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
//					HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);						// BOOST2
//					HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
					
					duty_cycle_BUCK1 = 0;
					duty_cycle_BUCK2 = 0;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;					
				} else {
					// Stop BUCK1 & BUCK2 & BOOST1 & BOOST2
					HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);						// BUCK1
					HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);						// BUCK2
					HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
					
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);							// BOOST1
					HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);							// BOOST2
					HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);					
					duty_cycle_BUCK1 = 0;
					duty_cycle_BUCK2 = 0;
					duty_cycle_BOOST1 = 0;
					duty_cycle_BOOST2 = 0;
				}
  			
  			SetDutyCircle_BUCK1(duty_cycle_BUCK1, 1);
  			SetDutyCircle_BUCK2(duty_cycle_BUCK2, 1);
  			SetDutyCircle_BOOST1(duty_cycle_BOOST1, 1);
  			SetDutyCircle_BOOST2(duty_cycle_BOOST2, 1);
#ifdef DEBUG
  			memset(buffer_case, 0, sizeof(buffer_case));
  			snprintf(buffer_case, 40, "current_case = %s\r\n", List_Case[current_case]);
  			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_case, strlen(buffer_case), HAL_MAX_DELAY);
#endif
  			previous_case = current_case;
				if (current_case != caseD5 && current_case != caseU15) HAL_Delay(1000);
			}
		}
		flow_chart_ebike_stage();
		query_data(); // Query all data eg ADC, I2C, voltage, current
		received_command();	// handle data send in UART/CAN

		if (counter_CAN_BMS >= 100) {		// every 1s, read data from BMS0 and BMS1
//			send_data();				// Send data to monitoring tools
			if (flag_BMS0_read == 1) {
				cur_time_BMS0 = HAL_GetTick();
				if ((cur_time_BMS0 - pre_time_BMS0) > FDCAN_TX_DELAY_MS) {
					flag_BMS0_read = CAN_BMS0_read_data();
					pre_time_BMS0 = cur_time_BMS0;
				}					
			}

			if (flag_BMS1_read == 1) {
				cur_time_BMS1 = HAL_GetTick();
				if ((cur_time_BMS1 - pre_time_BMS1) > FDCAN_TX_DELAY_MS) {
					flag_BMS1_read = CAN_BMS1_read_data();
					pre_time_BMS1 = cur_time_BMS1;
				}					
			}		
			if (flag_BMS0_read == 0 && flag_BMS1_read == 0 ) {
				// when read BMS0 and BMS1 completely, check BMS alive or not
				AuxBattery0_Info[26] = flag_BMS0_alive;
				AuxBattery1_Info[26] = flag_BMS1_alive;
				counter_CAN_BMS = 0;
				flag_BMS0_read = 1;
				flag_BMS1_read = 1;
				flag_BMS0_alive = 0;
				flag_BMS1_alive = 0;
			}
		}
				
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 9;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.NbrOfConversion = 2;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 32;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 4;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 32;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 4;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 32;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 4;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 32;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 4;
  hfdcan2.Init.DataTimeSeg2 = 5;
  hfdcan2.Init.StdFiltersNbr = 12;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30D29DE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1599;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1599;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1599;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
