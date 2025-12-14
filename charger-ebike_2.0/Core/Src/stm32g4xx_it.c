/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "charger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int counter = 0;
int counter_UART2 = 0;
int counter_UART3 = 0;
int counter_CAN_BMS = 0;
// Variable counter_Chg_Current_Adc_over_target is used to check the Chg_Current_Adc value 
// exceeds the target value threshold for how long, when the time threshold is exceeded, 
// PWM will decrease to 0
int counter_Chg_Current_Adc_over_target = 0;
int pwm_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Blink_Led(int specified_case);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc4;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	static int counter_ADC2 = 0;
	static uint16_t u16_ADC2_Val_temp[9] = {0};
	counter_ADC2++;
	if (counter_ADC2 <= 10) {
		for (int i = 0; i < 9; i++)
			u16_ADC2_Val_temp[i] += u16_ADC2_Val[i];
	} else {
		for (int i = 0; i < 9; i++) {
			u16_ADC2_Val_average[i] = u16_ADC2_Val_temp[i] / 10;
			u16_ADC2_Val_temp[i] = 0;
		}
		counter_ADC2 = 0;
	}
	
	
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	static int counter_ADC1 = 0;
	static uint16_t u16_ADC1_Val_temp[2] = {0};
	counter_ADC1++;
	if (counter_ADC1 <= 10) {
		for (int i = 0; i < 2; i++)
			u16_ADC1_Val_temp[i] += u16_ADC1_Val[i];
	} else {
		for (int i = 0; i < 2; i++) {
			u16_ADC1_Val_average[i] = u16_ADC1_Val_temp[i] / 10;
			u16_ADC1_Val_temp[i] = 0;
		}
		counter_ADC1 = 0;
	}
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	static int counter_ADC4 = 0;
	static uint16_t u16_ADC4_Val_temp[2] = {0};
	counter_ADC4++;
	if (counter_ADC4 <= 10) {
		for (int i = 0; i < 2; i++)
			u16_ADC4_Val_temp[i] += u16_ADC4_Val[i];
	} else {
		for (int i = 0; i < 2; i++) {
			u16_ADC4_Val_average[i] = u16_ADC4_Val_temp[i] / 10;
			u16_ADC4_Val_temp[i] = 0;
		}
		counter_ADC4 = 0;
	}
  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc4);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

	counter++;
	counter_UART2++;
	counter_CAN_BMS++;
	Blink_Led(current_case);
	if (counter == 10) {
		hardware_protection_flag = READ_HARD_PROTECTION;
		EV1 = hardware_protection_flag;
		if (hardware_protection_flag == high)
			next_stage = stage1;
	} else if (counter == 50) {
		// check voltage/ current
	} else if (counter == 100) {
		// reserved
	} else if (counter == 200) {
		counter = 0;
		// temp_NTC1, temp_NTC2, temp_NTC3, temp_NTC4;
		if (temp_NTC1 > 100.0 || temp_NTC2 > 100.0 || temp_NTC3 > 100.0 || temp_NTC4 > 100.0) {
			// over 100 degree Celcius
			temp_NTCx_status = 2;
		} else if (temp_NTC1 < -10.0 || temp_NTC2 < -10.0 || temp_NTC3 < -10.0 || temp_NTC4 < -10.0) {
			// less than -10 degree Celcius
			temp_NTCx_status = 3;
		} else if (temp_NTC1 < 85 && temp_NTC2 < 85 && temp_NTC3 < 85 && temp_NTC4 < 85 ) {
			// 0 (degree Celcius) < NTCx < 85 (degree Celcius)
			temp_NTCx_status = 0;
		} else if ((temp_NTC1 >= 85.0 && temp_NTC1 <= 100.0) ||
               (temp_NTC2 >= 85.0 && temp_NTC2 <= 100.0) ||
               (temp_NTC3 >= 85.0 && temp_NTC3 <= 100.0) ||
               (temp_NTC4 >= 85.0 && temp_NTC4 <= 100.0)) {
        temp_NTCx_status = 1;
    } else {
			// if all condition is wrong, hardcode temp_NTCx_status = 2
			temp_NTCx_status = 2;
    }
	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
	static int j = 0;
	int Step_Time_count_final = Step_Time_Count * Bat_Wakeup_Time_count + 3;
	// check temperature
	// Temp : over 85  --> no more PWM increase
	if (temp_NTCx_status == 1) return;
	// Temp : over 100'C  -->PWM duty decrease
	if (temp_NTCx_status == 2) {
		duty_cycle_BUCK1 = 0;
		duty_cycle_BUCK2 = 0;
		duty_cycle_BOOST1 = 0;
		duty_cycle_BOOST2 = 0;
		SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
		SetDutyCircle_BUCK2(duty_cycle_BUCK2, 0);
		SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
		SetDutyCircle_BOOST2(duty_cycle_BOOST2, 0);
		return;
	}
	// Temp : less than -10'C --> the max charge current 3A
	if (temp_NTCx_status == 3) {
		if (Chg_Current_Adc > Chg_Current_Target_Mid) {
			duty_cycle_BUCK1 = 0;
			duty_cycle_BUCK2 = 0;
			duty_cycle_BOOST1 = 0;
			duty_cycle_BOOST2 = 0;
			SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
			SetDutyCircle_BUCK2(duty_cycle_BUCK2, 0);
			SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
			SetDutyCircle_BOOST2(duty_cycle_BOOST2, 0);
			return;
		}
	}
	j++;
	counter_UART3++;
	
	if ( j > Step_Time_count_final) {
		float dt = Feed_Time_Basic * Step_Time_count_final;
		switch (current_case) {
			/**************************************/
			/************* BUCK MODE **************/
			/**************************************/	
			case caseD1:
				if (Chg_Current_Adc <= Chg_Current_Target_Low) {
					duty_cycle_BUCK1 = PID_Compute(&pid_caseD1, Chg_Current_Target_Low, Chg_Current_Adc, dt);
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				} else {
					// duty 1 same as previous
					duty_cycle_BUCK1 = 0;
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				}
				break;
				
			case caseD2:
				if (Chg_Current_Adc <= (Chg_Current_Target_Low * 0.95) || Chg_Current_Adc >= (Chg_Current_Target_Low * 1.05)) {
					duty_cycle_BUCK1 = PID_Compute(&pid_caseD2, Chg_Current_Target_Low, Chg_Current_Adc, dt);
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				} else {
					// duty 1 same as previous
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				}
				break;
			case caseD3:
				if (Chg_Current_Adc <= Chg_Current_Target_Mid * 0.95) {
					duty_cycle_BUCK1 = PID_Compute(&pid_caseD3, Chg_Current_Target_Mid, Chg_Current_Adc, dt);
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				}	else if (Chg_Current_Adc >= Chg_Current_Target_Mid * 1.05) {
					duty_cycle_BUCK1 = PID_Compute(&pid_caseD3, Chg_Current_Target_Mid, Chg_Current_Adc, dt);
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				} else {
					// duty 1 same as previous
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				}
				break;
			case caseD4:
				if (Phase1_Cur_Adc > Phase2_Cur_Adc) {
					float tmp_val = Phase1_Cur_Adc - Phase2_Cur_Adc;
					if (tmp_val < 0.075)
						Pwm_Top_Diff_Step_count = 0;
					else if (tmp_val < 0.11)
						Pwm_Top_Diff_Step_count = -1;
					else if (tmp_val < 0.15)
						Pwm_Top_Diff_Step_count = -2;
					else if (tmp_val < 0.3)
						Pwm_Top_Diff_Step_count = -3;
					else
						Pwm_Top_Diff_Step_count = -4;
				} else {
					float tmp_val = Phase2_Cur_Adc - Phase1_Cur_Adc;
					if (tmp_val < 0.075)
						Pwm_Top_Diff_Step_count = 0;
					else if (tmp_val < 0.11)
						Pwm_Top_Diff_Step_count = 1;
					else if (tmp_val < 0.15)
						Pwm_Top_Diff_Step_count = 2;
					else if (tmp_val < 0.3)
						Pwm_Top_Diff_Step_count = 3;
					else
						Pwm_Top_Diff_Step_count = 4;
				}
				
				Pwm_Top_Diff_Adj = Pwm_Step_Up_Down_Basic * Pwm_Top_Diff_Step_count;
				
				if (Chg_Current_Adc <= Chg_CC_Current * 0.95) {
					pwm_state = PWM_increase;
					counter_Chg_Current_Adc_over_target = 0;
//					Pwm_Step_Up_Down_Phase1_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase1_Cur, Chg_CC_Current/2, Phase1_Cur_Adc, dt);
//					Pwm_Step_Up_Down_Phase2_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase2_Cur, Chg_CC_Current/2, Phase2_Cur_Adc, dt);
					Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseD4, Chg_CC_Current, Chg_Current_Adc, dt);
					if (Pwm_Step_Up_Down < 0) Pwm_Step_Up_Down = 0;
					
					Buck_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;

					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down + Buck_Pwm_Top_Diff_Adj;
					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down;
//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Phase1_Cur + Buck_Pwm_Top_Diff_Adj;
//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Phase2_Cur;
//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state + Buck_Pwm_Top_Diff_Adj;
//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state;
					SetPWM_BUCK1(Buck_Pwm_Top1, 0);
					SetPWM_BUCK2(Buck_Pwm_Top2, 0);
				} else if (Chg_Current_Adc >= Chg_CC_Current * 1.05) {
					pwm_state = PWM_decrease;
					counter_Chg_Current_Adc_over_target++;
					if (counter_Chg_Current_Adc_over_target >= 75000) {
						counter_Chg_Current_Adc_over_target = 75000;
						// when Chg_Current_Adc over Chg_CC_Current more than 3s, 
						// set PWM decrease to 0 quickly, not use PID
						Buck_Pwm_Top1 = 0;
						Buck_Pwm_Top2 = 0;
					} else {
						
	//					Pwm_Step_Up_Down_Phase1_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase1_Cur, Chg_CC_Current/2, Phase1_Cur_Adc, dt);
	//					Pwm_Step_Up_Down_Phase2_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase2_Cur, Chg_CC_Current/2, Phase2_Cur_Adc, dt);
						Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseD4, Chg_CC_Current, Chg_Current_Adc, dt);
						if (Pwm_Step_Up_Down < 0) Pwm_Step_Up_Down = 0;
						Buck_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;
						
	//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down + Buck_Pwm_Top_Diff_Adj;
	//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down;
						Buck_Pwm_Top1 = Pwm_Step_Up_Down + Buck_Pwm_Top_Diff_Adj;
						Buck_Pwm_Top2 = Pwm_Step_Up_Down;
	//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Phase1_Cur + Buck_Pwm_Top_Diff_Adj;
	//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Phase2_Cur;
	//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state + Buck_Pwm_Top_Diff_Adj;
	//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state;
					}
					SetPWM_BUCK1(Buck_Pwm_Top1, 0);
					SetPWM_BUCK2(Buck_Pwm_Top2, 0);
				}
				else {
					// duty BUCK1&2 same as previous
					pwm_state = PWM_hold;
					counter_Chg_Current_Adc_over_target = 0;
					SetPWM_BUCK1(Buck_Pwm_Top1, 0);
					SetPWM_BUCK2(Buck_Pwm_Top2, 0);
				}
				duty_cycle_BUCK1 = PWM_counter_BUCK1 / 1599.0 * 100;
				duty_cycle_BUCK2 = PWM_counter_BUCK2 / 1599.0 * 100;

				break;
			case caseD5:
				if (Main_Chg_Batin_Adc < 2.4) {
					if (Phase1_Cur_Adc > Phase2_Cur_Adc) {
						float tmp_val = Phase1_Cur_Adc - Phase2_Cur_Adc;
						if (tmp_val < 0.075)
							Pwm_Top_Diff_Step_count = 0;
						else if (tmp_val < 0.11)
							Pwm_Top_Diff_Step_count = -1;
						else if (tmp_val < 0.15)
							Pwm_Top_Diff_Step_count = -2;
						else if (tmp_val < 0.3)
							Pwm_Top_Diff_Step_count = -3;
						else
							Pwm_Top_Diff_Step_count = -4;
					} else {
						float tmp_val = Phase2_Cur_Adc - Phase1_Cur_Adc;
						if (tmp_val < 0.075)
							Pwm_Top_Diff_Step_count = 0;
						else if (tmp_val < 0.11)
							Pwm_Top_Diff_Step_count = 1;
						else if (tmp_val < 0.15)
							Pwm_Top_Diff_Step_count = 2;
						else if (tmp_val < 0.3)
							Pwm_Top_Diff_Step_count = 3;
						else
							Pwm_Top_Diff_Step_count = 4;
					}
					
					Pwm_Top_Diff_Adj = Pwm_Step_Up_Down_Basic * Pwm_Top_Diff_Step_count;
					
					Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseD5, 2.40, Main_Chg_Batin_Adc, dt);
					if (Pwm_Step_Up_Down < 0) Pwm_Step_Up_Down = 0;
					Buck_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;
					
//					Buck_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down + Buck_Pwm_Top_Diff_Adj;
//					Buck_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down;
					
					Buck_Pwm_Top1 = Pwm_Step_Up_Down + Buck_Pwm_Top_Diff_Adj;
					Buck_Pwm_Top2 = Pwm_Step_Up_Down;
					SetPWM_BUCK1(Buck_Pwm_Top1, 0);
					SetPWM_BUCK2(Buck_Pwm_Top2, 0);
				} else {
					// duty 1 same as previous
					SetPWM_BUCK1(Buck_Pwm_Top1, 0);
					SetPWM_BUCK2(Buck_Pwm_Top2, 0);
				}
				duty_cycle_BUCK1 = PWM_counter_BUCK1 / 1599.0 * 100;
				duty_cycle_BUCK2 = PWM_counter_BUCK2 / 1599.0 * 100;
				break;
			case caseD6:
				if (Main_Chg_Batin_Adc < 2.4 ||  Main_Chg_Batin_Adc > 2.45) {
					duty_cycle_BUCK1 = PID_Compute(&pid_caseD6, 2.45, Main_Chg_Batin_Adc, dt);
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				} else {
				// duty 1 same as previous
					SetDutyCircle_BUCK1(duty_cycle_BUCK1, 0);
				}
				break;
			/**************************************/
			/************* BOOST MODE *************/
			/**************************************/				
			case caseU11:
				if (Chg_Current_Adc <= Chg_Current_Target_Low) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU11, Chg_Current_Target_Low, Chg_Current_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else {
					// duty 1 same as previous
					duty_cycle_BOOST1 = 0;
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				}
				break;
			case caseU12:
				if (Chg_Current_Adc <= (Chg_Current_Target_Low * 0.95)) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU12, Chg_Current_Target_Low, Chg_Current_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else if (Chg_Current_Adc >= (Chg_Current_Target_Low * 1.05)) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU12, Chg_Current_Target_Low, Chg_Current_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else {
					// duty 1 same as previous
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				}
				break;
			case caseU13:
				if (Chg_Current_Adc <= (Chg_Current_Target_Mid * 0.95)) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU13, Chg_Current_Target_Mid, Chg_Current_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else if (Chg_Current_Adc >= (Chg_Current_Target_Mid * 1.05)) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU13, Chg_Current_Target_Mid, Chg_Current_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else {
					// duty 1 same as previous
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				}
				break;
			case caseU14:
				if (Phase1_Cur_Adc > Phase2_Cur_Adc) {
					float tmp_val = Phase1_Cur_Adc - Phase2_Cur_Adc;
					if (tmp_val < 0.075)
						Pwm_Top_Diff_Step_count = 0;
					else if (tmp_val < 0.11)
						Pwm_Top_Diff_Step_count = -1;
					else if (tmp_val < 0.15)
						Pwm_Top_Diff_Step_count = -2;
					else if (tmp_val < 0.3)
						Pwm_Top_Diff_Step_count = -3;
					else
						Pwm_Top_Diff_Step_count = -4;
				} else {
					float tmp_val = Phase2_Cur_Adc - Phase1_Cur_Adc;
					if (tmp_val < 0.075)
						Pwm_Top_Diff_Step_count = 0;
					else if (tmp_val < 0.11)
						Pwm_Top_Diff_Step_count = 1;
					else if (tmp_val < 0.15)
						Pwm_Top_Diff_Step_count = 2;
					else if (tmp_val < 0.3)
						Pwm_Top_Diff_Step_count = 3;
					else
						Pwm_Top_Diff_Step_count = 4;
				}
				
				Pwm_Top_Diff_Adj = Pwm_Step_Up_Down_Basic * Pwm_Top_Diff_Step_count;
				
				if (Chg_Current_Adc <= Chg_CC_Current * 0.95) {
					pwm_state = PWM_increase;
					counter_Chg_Current_Adc_over_target = 0;
//					Pwm_Step_Up_Down_Phase1_Cur = PID_Compute_PWM_adj(&pid_caseU14_Phase1_Cur, Chg_CC_Current/2, Phase1_Cur_Adc, dt);
//					Pwm_Step_Up_Down_Phase2_Cur = PID_Compute_PWM_adj(&pid_caseU14_Phase2_Cur, Chg_CC_Current/2, Phase2_Cur_Adc, dt);
					Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseU14, Chg_CC_Current, Chg_Current_Adc, dt);
					
					Boost_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;

					Boost_Pwm_Top1 = Pwm_Step_Up_Down + Boost_Pwm_Top_Diff_Adj;
					Boost_Pwm_Top2 = Pwm_Step_Up_Down;
//					Boost_Pwm_Top1 = Boost_Pwm_Top_Start + Pwm_Step_Up_Down_Phase1_Cur + Boost_Pwm_Top_Diff_Adj;
//					Boost_Pwm_Top2 = Boost_Pwm_Top_Start + Pwm_Step_Up_Down_Phase2_Cur;
//					Boost_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state + Boost_Pwm_Top_Diff_Adj;
//					Boost_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state;
					SetPWM_BOOST1(Boost_Pwm_Top1, 0);
					SetPWM_BOOST2(Boost_Pwm_Top2, 0);
				} else if (Chg_Current_Adc >= Chg_CC_Current * 1.05) {
					pwm_state = PWM_decrease;
					counter_Chg_Current_Adc_over_target++;
					if (counter_Chg_Current_Adc_over_target >= 75000) {
						counter_Chg_Current_Adc_over_target = 75000;
						// when Chg_Current_Adc over Chg_CC_Current more than 3s, 
						// set PWM decrease to 0 quickly, not use PID
						Boost_Pwm_Top1 = 0;
						Boost_Pwm_Top2 = 0;
					} else {
						// Pwm_Step_Up_Down_Phase1_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase1_Cur, Chg_CC_Current/2, Phase1_Cur_Adc, dt);
						// Pwm_Step_Up_Down_Phase2_Cur = PID_Compute_PWM_adj(&pid_caseD4_Phase2_Cur, Chg_CC_Current/2, Phase2_Cur_Adc, dt);
						Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseU14, Chg_CC_Current, Chg_Current_Adc, dt);
						
						Boost_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;
						
						Boost_Pwm_Top1 = Pwm_Step_Up_Down + Boost_Pwm_Top_Diff_Adj;
						Boost_Pwm_Top2 = Pwm_Step_Up_Down;
						// Boost_Pwm_Top1 = Boost_Pwm_Top_Start + Pwm_Step_Up_Down_Phase1_Cur + Boost_Pwm_Top_Diff_Adj;
						// Boost_Pwm_Top2 = Boost_Pwm_Top_Start + Pwm_Step_Up_Down_Phase2_Cur;
						// Boost_Pwm_Top1 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state + Boost_Pwm_Top_Diff_Adj;
						// Boost_Pwm_Top2 = Buck_Pwm_Top_Start + Pwm_Step_Up_Down_Basic * pwm_state;
					}

					SetPWM_BOOST1(Boost_Pwm_Top1, 0);
					SetPWM_BOOST2(Boost_Pwm_Top2, 0);
				}
				else {
					// duty BOOST1&2 same as previous
					pwm_state = PWM_hold;
					counter_Chg_Current_Adc_over_target = 0;
					SetPWM_BOOST1(Boost_Pwm_Top1, 0);
					SetPWM_BOOST2(Boost_Pwm_Top2, 0);
				}
				duty_cycle_BOOST1 = PWM_counter_BOOST1 / 1599.0 * 100;
				duty_cycle_BOOST2 = PWM_counter_BOOST2 / 1599.0 * 100;
				break;
			case caseU15:
				if (Main_Chg_Batin_Adc < 2.4) {
					if (Phase1_Cur_Adc > Phase2_Cur_Adc) {
						float tmp_val = Phase1_Cur_Adc - Phase2_Cur_Adc;
						if (tmp_val < 0.075)
							Pwm_Top_Diff_Step_count = 0;
						else if (tmp_val < 0.11)
							Pwm_Top_Diff_Step_count = -1;
						else if (tmp_val < 0.15)
							Pwm_Top_Diff_Step_count = -2;
						else if (tmp_val < 0.3)
							Pwm_Top_Diff_Step_count = -3;
						else
							Pwm_Top_Diff_Step_count = -4;
					} else {
						float tmp_val = Phase2_Cur_Adc - Phase1_Cur_Adc;
						if (tmp_val < 0.075)
							Pwm_Top_Diff_Step_count = 0;
						else if (tmp_val < 0.11)
							Pwm_Top_Diff_Step_count = 1;
						else if (tmp_val < 0.15)
							Pwm_Top_Diff_Step_count = 2;
						else if (tmp_val < 0.3)
							Pwm_Top_Diff_Step_count = 3;
						else
							Pwm_Top_Diff_Step_count = 4;
					}
					
					Pwm_Top_Diff_Adj = Pwm_Step_Up_Down_Basic * Pwm_Top_Diff_Step_count;
					
					Pwm_Step_Up_Down = PID_Compute_PWM_adj(&pid_caseU15, 2.40, Main_Chg_Batin_Adc, dt);
					
					Boost_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_count;
					
					Boost_Pwm_Top1 = Pwm_Step_Up_Down + Boost_Pwm_Top_Diff_Adj;
					Boost_Pwm_Top2 = Pwm_Step_Up_Down;
					
					SetPWM_BOOST1(Boost_Pwm_Top1, 0);
					SetPWM_BOOST2(Boost_Pwm_Top2, 0);
				} else {
					// duty BOOST1&2 same as previous
					SetPWM_BOOST1(Boost_Pwm_Top1, 0);
					SetPWM_BOOST2(Boost_Pwm_Top2, 0);
				}
				duty_cycle_BOOST1 = PWM_counter_BOOST1 / 1599.0 * 100;
				duty_cycle_BOOST2 = PWM_counter_BOOST2 / 1599.0 * 100;
				break;
			case caseU16:
				if (Main_Chg_Batin_Adc < 2.4 ||  Main_Chg_Batin_Adc > 2.45) {
					duty_cycle_BOOST1 = PID_Compute(&pid_caseU16, 2.45, Main_Chg_Batin_Adc, dt);
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				} else {
				// duty 1 same as previous
					SetDutyCircle_BOOST1(duty_cycle_BOOST1, 0);
				}
				break;
			default:
				break;
		}
		j = 1;
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Blink_Led(int specified_case)
{
	static int i = 0;
	int repeat_blink = 0, period_blink = 0;

	switch (specified_case) {
		case Input_Power_on:
			// Led 0.5s on 0.5s off
			if (counter % 50 == 0) TOGGLE_LED;
			break;
		// Stage 6 in promode
		case caseD1:
			// Led 0.25s on 0.25s off 2 times repeat on /off 0.5sec off
			repeat_blink = 2;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseD2:
			// Led 0.25s on 0.25s off 3 times repeat on /off 0.5sec off
			repeat_blink = 3;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseD3:
			// Led 0.25s on 0.25s off 4 times repeat on /off 0.5sec off
			repeat_blink = 4;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseD4:
			// Led 0.25s on 0.25s off 5 times repeat on /off 0.5sec off
			repeat_blink = 5;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseD5:
			// Led 0.25s on 0.25s off 6 times repeat on /off 0.5sec off
			repeat_blink = 6;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseD6:
			// Led 0.25s on 0.25s off 7 times repeat on /off 0.5sec off
			repeat_blink = 7;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		// Stage 7 in promode
		case caseU11:
			// Led 0.25s on 0.25s off 9 times repeat on /off 0.5sec off
			repeat_blink = 9;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseU12:
			// Led 0.25s on 0.25s off 10 times repeat on /off 0.5sec off
			repeat_blink = 10;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseU13:
			// Led 0.25s on 0.25s off 11 times repeat on /off 0.5sec off
			repeat_blink = 11;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseU14:
			// Led 0.25s on 0.25s off 12 times repeat on /off 0.5sec off
			repeat_blink = 12;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseU15:
			// Led 0.25s on 0.25s off 13 times repeat on /off 0.5sec off
			repeat_blink = 13;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseU16:
			// Led 0.25s on 0.25s off 14 times repeat on /off 0.5sec off
			repeat_blink = 14;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		// Stage 8 in promode
		case caseA21:
			// Led 0.25s on 0.25s off 15 times repeat on /off 0.5sec off
			repeat_blink = 15;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseA22:
			// Led 0.25s on 0.25s off 16 times repeat on /off 0.5sec off
			repeat_blink = 16;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseA23:
			// Led 0.25s on 0.25s off 17 times repeat on /off 0.5sec off
			repeat_blink = 17;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		case caseA24:
			// Led 0.25s on 0.25s off 18 times repeat on /off 0.5sec off
			repeat_blink = 18;
			period_blink = 2 * repeat_blink + 2;
			if (counter % 25 == 0) {
				i++;
				if (i <= 2 * repeat_blink) TOGGLE_LED;
				else if (i <= period_blink) TURNOFF_LED;
				else {
					i = 1;
					TURNON_LED;
				}
			}
			break;
		// Stage 9 in promode
		case caseA31:
		default:
			TURNON_LED;
			break;
	}
}
/* USER CODE END 1 */
