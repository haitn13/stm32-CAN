#include "charger.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"

// System Variable
//ADC_HandleTypeDef hadc1;
//ADC_HandleTypeDef hadc2;
//ADC_HandleTypeDef hadc4;

//FDCAN_HandleTypeDef hfdcan1;
//FDCAN_HandleTypeDef hfdcan2;

//I2C_HandleTypeDef hi2c3;

//IWDG_HandleTypeDef hiwdg;

//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim8;
//TIM_HandleTypeDef htim15;
//TIM_HandleTypeDef htim17;

//UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart3;


PID_Controller pid_caseD1, pid_caseD2, pid_caseD3, \
								pid_caseD4, pid_caseD4_Phase1_Cur, pid_caseD4_Phase2_Cur, \
								pid_caseD5, pid_caseD5_Phase1_Cur, pid_caseD5_Phase2_Cur, \
								pid_caseD6;
								
PID_Controller pid_caseU11, pid_caseU12, pid_caseU13, \
					pid_caseU14, pid_caseU14_Phase1_Cur, pid_caseU14_Phase2_Cur, \
					pid_caseU15, pid_caseU15_Phase1_Cur, pid_caseU15_Phase2_Cur, \
					pid_caseU16;

// Charger Variable
char buffer_debug[80] = {0};
char *List_Stage[] = {
	"stage0",
	"stage1",
	"stage2",
	"stage3",
	"stage4",
	"stage5",
	"stage6",
	"stage7",
	"stage8",
	"stage9",
	"stageA1",
	"stageA31",
	"stageA32",
	"stageA33",
	"stageA34",
	"laststage"
};
char *List_Case[] = {
	"Input_Power_on",
	// Stage 6 in promode
	"caseD1",
	"caseD2",
	"caseD3",
	"caseD4",
	"caseD5",
	"caseD6",
	"caseD7",		// Reserved
	"caseD8",		// Reserved
	"caseD9",		// Reserved
	"caseD10",
	// Stage 7 in promode
	"caseU11",
	"caseU12",
	"caseU13",
	"caseU14",
	"caseU15",
	"caseU16",
	"caseU17",		// Reserved
	"caseU18",		// Reserved
	"caseU19",		// Reserved
	"caseU20",		// Reserved
	// Stage 8 in promode
	"caseA21",
	"caseA22",
	"caseA23",
	"caseA24",
	// Stage 9 in promode
	"caseA31",
	"lastcase"
};

int next_stage = 0;
int current_stage = 0;					// this is flag for stage in flow chart
int previous_stage = 0;
int current_case = 0;						// this is flag for CASE inf flow chart
int previous_case = 0;
uint32_t set_delay = 70000, init_time = 0, current_time = 0;

uint8_t flag1_D6 = 0, flag2_D6 = 0;
uint8_t flag1_U16 = 0, flag2_U16 = 0;

uint32_t init_time_D6 = 0, current_time_D6 = 0;
uint32_t init_time_D5 = 0, current_time_D5 = 0;
uint32_t init_time_U16 = 0, current_time_U16 = 0;
uint32_t init_time_U15 = 0, current_time_U15 = 0;

uint8_t AC_Respond_flag = 0, AuxBattery_Respond_flag = 0;

uint8_t Initial_Start = 0;
int hardware_protection_flag = 0, software_protection_flag = 0;

float Vcheck_12V = 0;
float Vcheck_5V = 0;

// CHG_CURRENT - 
float Chg_Current_Adc = 0.0;
float Chg_High_Current_Adc = 0.0;
float Phase1_Cur_Adc = 0.0;
float Phase2_Cur_Adc = 0.0;
float Chg_Cur_Target = 1.92;
float Chg_Current_Target_Low = 0.12;				// Chg_Current_Target_Low(1.2A) : 0.12V
float Chg_Current_Target_Mid = 0.28;				// Chg_Current_Target_Mid(3A) : 0.28V
float Chg_Current_Lim_Max = 2.4;					// Chg_Current_Lim_Max(25A) : 2.4V 
float Chg_Phase_Current_Lim_Max = 3.2;				// phase 1 or phase 2 max current
float Chg_CC_Current = 1.95;						// phase 1+ phase 2 : target current, Chg_CC_Current(20A): 1.95V
uint8_t Phase_Mode = 1;								// single phase or dual phase
float Chg_Phase_Current = 0.0;						// need calculate
float Chg_Phase_Current_CV_End = 2.52;
float Chg_Current_CV_End = 0.03;
float Chg_Phase_Current_Max = 0.0;	// need calculate
 
// MODE_SELECTION 
uint8_t Mode_Define_Power = 1;
uint8_t Mode_Define_HD = 1;
uint8_t Buck_Mode = 0;
uint8_t Boost_Mode = 0;


// MAIN_CHG_BATVIN
float Main_Chg_Batin_Adc = 0.0;
float Chg_Target_Vout = 0.0;						// Vout set of Main battery
float Main_Bat_Input_Max = 2.5;					// Main_Bat_Input_Max(84V - 20*cell 4.2V/cell): 2.5V
float Main_Bat_CV = 2.37;							// Main_Bat_CV(81V - 20*cell 4.06V/cell): 2.37V
float Main_Bat_CV_CC = 2.27;						// Main_Bat_CV_CC(78V - 20*cell 3.9V/cell): 2.27
float Main_Bat_CV_CC_Max = 2.45;					// Main_Bat_CV_CC_Max(84V - 20*cell 4.2V/cell): 2.45;
float BAT_Chg_Trickle_Vout = 1.92;						// BAT_Chg_Trickle_Vout (66V - 20*cell 3.3V/cell): 1.92;
float BAT_Chg_Wake_Vout = 1.75;						// BAT_Chg_Wake_Vout (60V - 20*cell 3.0V/cell): 1.75;

// ACCHG_BAT_VIN
float ACchg_Bat_Vin_Adc = 0.0;

// AUX_BAT_VIN
float Aux_Bat_Vin_Adc = 0.0;
float Aux_Bat_Min = 0.0;						// buck mode : Aux_Bat_Min = Aux_Bat_Min_28 or Boost Mode : Aux_Bat_Min = Aux_Bat_Min_14
float Aux_Bat_Max = 0.0;						// buck mode : Aux_Bat_Max = Aux_Bat_Max_28 or Boost Mode : Aux_Bat_Max = Aux_Bat_Max_14
float Aux_Bat_Max_28 = 2.3;
float Aux_Bat_Max_14 = 1.15;
float Aux_Bat_Min_28 = 1.64;
float Aux_Bat_Min_14 = 0.82;


int Pwm_Freq = 100000;							// Pwm_Freq = 100kHZ
float Feed_Time_Basic = 0.00001;		// Feed_Time_Basic = 1 / Pwm_Freq
int Pwm_Start_Slew_Rate = 100;			// 100ms
int Feed_Back_Time_Int = 3;
int Feed_Back_Time = 0;						// need calculate

int Pwm_Basic_Step = 1;						// one basic increase step is 20 step. max step = 2^16 = 65536
																	// Update one basic increase step is 5 step. max step = 1600
int Pwm_Step_Count = 1;						// 
int Pwm_Step_Up_Down_Basic = 1;		// Pwm_Step_Up_Down_Basic = Pwm_Basic_Step * Pwm_Step_Count

int Buck_Pwm_Top_Start = 640;		// 40% of 65535 when use 16 bit timer
																	// Update 40% of 1600 = 640
int Pwm_Top_Diff_Adj = 6;					// need calculate, Pwm_Top_Diff_Adj = Pwm_Step_Up_Down_Basic * Pwm_Top_Diff_Step_count
int Pwm_Top_Diff_Step_count = 2;	// need calculate, using software


int Step_Time_Count = 2;
int Step_Time_Int = 0; 						// Default Step_Time_Int = Feed_Time_Basic * Step_Time_Count * Bat_Wakeup_Time_count = 
int Bat_Wakeup_Time_count = 5;		// use only when battery Main wakeup

int Pwm_Step_Up_Down = 0;
// BUCK_TOP1
int Buck_Pwm_Top1 = 0;				// = Buck_Pwm_Top_Start +/-Pwm_Step_Up_Down* No( increase or decreae )+/-Buck_Pwm_Top_Diff_Adj
int Pwm_Step_Up_Down_Phase1_Cur = 0;
int Buck_Pwm_Top_Diff_Adj = 0;		// Buck_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_Count
// BUCK_BOT1
int Buck_Pwm_Bot1 = 0;

// BUCK_TOP2
int Buck_Pwm_Top2 = 0;				// = Buck_Pwm_Top_Start +/-Pwm_Step_Up_Down* No( increase or decreae )+/-Buck_Pwm_Top_Diff_Adj * zero( 0) 
									// ( up down decision by target current compare)
int Pwm_Step_Up_Down_Phase2_Cur = 0;
int Buck_Top2_Phase_Adj = 0;		// 0 = same phase
									// 1 = 180 degree difference phase between buck 1/ buck 2 ??
int Buck_Pwm_Bot2 = 0;


int Boost_Pwm_Top_Start = 640;		// 40% of 65535 when use 16 bit timer
																	// Update 40% of 1600 = 640
int Boost_Pwm_Top_Diff_Adj = 0;		// Boost_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_Count																	
// BOOST_BOT1
int Boost_Pwm_Bot1 = 0;
int Boost_Pwm_Bot1_Diff_Adj = 0;
int Boost_Pwm_Bot_Max = 800;		// 50% of 1600 = 800
// BOOST_TOP1
int Boost_Pwm_Top1 = 0;
int Boost_Top2_Phase_Adj = 0;		// 0 = same phase
									// 1 = 180 degree difference phase between boost 1/ boost 2 ??
// BOOST_TOP2
int Boost_Pwm_Top2 = 0;
// BOOST_BOT2
int Boost_Pwm_Bot2 = 0;

float temp_NTC1 = 0, temp_NTC2 = 0, temp_NTC3 = 0, temp_NTC4 = 0;
uint8_t temp_NTCx_status = 0;

int PulseOut = 0;
float Dutycycle_Setpoint = 0;

float duty_cycle_BUCK1 = 0;
float duty_cycle_BUCK2 = 0;
float duty_cycle_BOOST1 = 0;
float duty_cycle_BOOST2 = 0;

int PWM_counter_BUCK1 = 0;
int PWM_counter_BUCK2 = 0;
int PWM_counter_BOOST1 = 0;
int PWM_counter_BOOST2 = 0;

float real_duty_cycle_BUCK1 = 0;
float real_duty_cycle_BUCK2 = 0;
float real_duty_cycle_BOOST1 = 0;
float real_duty_cycle_BOOST2 = 0;

static uint8_t cnt_d1 = 0, cnt_d2 = 0, cnt_d3 = 0, cnt_d4 = 0, cnt_d5 = 0, cnt_d6 = 0;
static uint8_t cnt_u11 = 0, cnt_u12 = 0, cnt_u13 = 0, cnt_u14 = 0, cnt_u15 = 0, cnt_u16 = 0;
static uint8_t cnt_A21 = 0, cnt_A22 = 0, cnt_A23 = 0, cnt_A24 = 0;

void flow_chart_ebike_stage()
{
	current_stage = next_stage;
//	memset(buffer_debug, 0, sizeof(buffer_debug));
//	snprintf(buffer_debug, 80, "current_stage = %s\r\n", List_Stage[current_stage] );
//	HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);	
	switch (current_stage) {
		case stage0:
			Initial_Start = 1;
			next_stage = stage1;
			break;
		case stage1:
			/* Initialize */
			cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
			cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
			cnt_A21 = cnt_A22 = cnt_A23 = cnt_A24 = 0;
			flag1_D6 = flag2_D6 = 0;
			flag1_U16 = flag2_U16 = 0;
			// All output low
			// PWM
			HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);	
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
			
			duty_cycle_BUCK1 = 0;
			duty_cycle_BUCK2 = 0;
			duty_cycle_BOOST1 = 0;
			duty_cycle_BOOST2 = 0;
			SetDutyCircle_BUCK1(duty_cycle_BUCK1, 1);
			SetDutyCircle_BUCK2(duty_cycle_BUCK2, 1);
			SetDutyCircle_BOOST1(duty_cycle_BOOST1, 1);
			SetDutyCircle_BOOST2(duty_cycle_BOOST2, 1);

			AC_CHG_EN(LOW);
			MAIN_POWER_ON(LOW);
			BUCK_BOOST_OFF_P1(LOW);
			ANTI_REVERSE(LOW);
			BUCK_BOOST_OFF_P2(LOW);
			LED_CONTROL(LOW);
		
			HAL_Delay(1000);
			current_case = Input_Power_on;
			next_stage = stage2;
			init_time = HAL_GetTick();
			break;
		case stage2:
			/* DC voltage check */
			if (DCVoltageCheck() == bad)
				next_stage = stageA31;
			else {
				if (Main_Chg_Batin_Adc < 2.33) {
					next_stage = stage3;
				} else if (Main_Chg_Batin_Adc <= 2.39) {
					// Delay 10s and then goto stage3
					current_time = HAL_GetTick();
					if (current_time - init_time > 10000)
						next_stage = stage3;
				} else if (Main_Chg_Batin_Adc > 2.39) {
					init_time = HAL_GetTick();
					next_stage = stage2;
				}
			}
			break;
		case stage3:
			/* Mode define */
			Mode_Define_HD = READ_MODE_SELECTION;
			// when trigger protection of hard/software, comback stage1
			if (hardware_protection_flag == high) {
				next_stage = stage1;
			} else {
				init_time = HAL_GetTick();
				next_stage = stage4;
			}
			break;
		case stage4:
			if (Initial_Start == 0) {
				// Delay 70s and then goto stage4, set_delay = 70000 
				current_time = HAL_GetTick();
				if (current_time - init_time > set_delay) {
					software_protection_flag = 0;
					next_stage = stage5;
				}
			} else {
				Initial_Start = 0;
				next_stage = stage5;
			}
			break;
		case stage5:
			/* Power voltage check */
			if (ACchg_Bat_Vin_Adc > 2.5 || Aux_Bat_Vin_Adc > 2.5) {
				// When BAD thing happends, comback stage1
				EV3 = 1;
				next_stage = stage1;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (ACchg_Bat_Vin_Adc > 2.5) HAL_UART_Transmit(&huart3, (uint8_t *)"ACchg_Bat_Vin_Adc>2.5\r\n", strlen("ACchg_Bat_Vin_Adc>2.5\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc > 2.5) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc>2.5\r\n", strlen("Aux_Bat_Vin_Adc>2.5\r\n"), HAL_MAX_DELAY);
#endif
				break;
			}
			EV3 = 0;	
			//	Check whether or not communicate with AC power through CAN/RS485
			AC_Respond_flag = check_comunication_AC_power();
			if (AC_Respond_flag == 0 && ACchg_Bat_Vin_Adc < 0.7) {		
				Mode_Define_HD = READ_MODE_SELECTION;
				if (Mode_Define_HD == high) {
					// Buck mode
					Mode_Define_Power = high;
					next_stage = stage6;
				}	else if (Mode_Define_HD == low) {
					// Boost mode
					Mode_Define_Power = low;
					next_stage = stage7;
				}
				break;
			}
			
			if (AC_Respond_flag == 0 && ACchg_Bat_Vin_Adc >= 0.7) {
				next_stage = stage9;
				break;
			}
			
			if (AC_Respond_flag == 1) {
				if (ACchg_Bat_Vin_Adc < 0.7) {
					// when ACpower < 0.7, stay at stage 5
					set_ACpower_voltage(60);
					next_stage = stage5;
				} else {
					next_stage = stage8;
				}
			}
			break;
		case stage6:
			/* Enable Auxilarity DC power in Buck mode */
			// Checking voltage/current condition is normal or not
//			if (1.65 > Aux_Bat_Vin_Adc  ||  Aux_Bat_Vin_Adc > 2.35 || \
//					Main_Chg_Batin_Adc >= Main_Bat_Input_Max || Chg_Current_Adc >= Chg_Current_Lim_Max) {
			if (1.65 > Aux_Bat_Vin_Adc  ||  Aux_Bat_Vin_Adc > 2.35 || Chg_Current_Adc >= Chg_Current_Lim_Max) {
				// Except loop
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (1.65 > Aux_Bat_Vin_Adc) HAL_UART_Transmit(&huart3, (uint8_t *)"1.65>Aux_Bat_Vin_Adc\r\n", strlen("1.65>Aux_Bat_Vin_Adc\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc > 2.35) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc>2.35\r\n", strlen("Aux_Bat_Vin_Adc>2.35\r\n"), HAL_MAX_DELAY);
				if (Chg_Current_Adc >= Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>= Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>= Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
#endif
				break;
			}
			cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
			cnt_A21 = cnt_A22 = cnt_A23 = cnt_A24 = 0;
			
			// if main bat is over 2.48 : immediately charge compelte.
			if (Main_Chg_Batin_Adc > 2.48) {
				next_stage = stage1;
				Initial_Start = 1;
				break;
			}
			
			if (Main_Chg_Batin_Adc < 1.75) {
				if (1.65 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 2.35) {
					// Main Battery wake
					cnt_d1++;
					cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
					if (cnt_d1 >= 10) {
						current_case = caseD1;
						flow_chart_ebike_case(caseD1);
						cnt_d1 = 10;
					}
				}
			} else if (Main_Chg_Batin_Adc < 1.92) {
				if (1.65 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 2.35) {
					// Trickle Charging
					cnt_d2++;
					cnt_d1 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
					if (cnt_d2 >= 10) {
						current_case = caseD2;
						flow_chart_ebike_case(caseD2);
						cnt_d2 = 10;
					}
				}
			}	else if (Main_Chg_Batin_Adc < 2.33) {
				if (1.65 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.8) {
					// LP Charge mode
					cnt_d3++;
					cnt_d1 = cnt_d2 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
					if (cnt_d3 >= 10) {
						current_case = caseD3;
						flow_chart_ebike_case(caseD3);
						cnt_d3 = 10;
					}
				} else if (Aux_Bat_Vin_Adc < 1.81) {
					if (current_case == caseD3 || current_case == caseD4) {
						flow_chart_ebike_case(current_case);
					} else {
						current_case = caseD3;
						flow_chart_ebike_case(current_case);
					}
					
				} else if (1.81 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 2.3) {
					// Normal CC
					cnt_d4++;
					cnt_d1 = cnt_d2 = cnt_d3 = cnt_d5 = cnt_d6 = 0;
					if (cnt_d4 >= 10) {
						current_case = caseD4;
						flow_chart_ebike_case(caseD4);
						cnt_d4 = 10;
					}
				}
			}	else if (Main_Chg_Batin_Adc <= 2.45 && Chg_Current_Adc >= Chg_Current_Target_Low) {
				if (1.65 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 2.35) {
					// CV mode H
					cnt_d5++;
					cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d6 = 0;
					if (cnt_d5 >= 10) {
						current_case = caseD5;
						flow_chart_ebike_case(caseD5);
						cnt_d5 = 10;
					}
					init_time_D5 = HAL_GetTick();
				}
			} else if (current_case == caseD5) {
				if (Main_Chg_Batin_Adc <= 2.48 || Chg_Current_Adc < Chg_Current_Target_Low) {
					current_time_D5 = HAL_GetTick();
					if (current_time_D5 - init_time_D5 > 3000) {
						current_case = caseD6;
						flow_chart_ebike_case(caseD6);
					}
				} else {
					current_case = caseD6;
					flow_chart_ebike_case(caseD6);
				}
				
			} else if (Chg_Current_Adc >= Chg_Current_CV_End) {
//			} else if (Main_Chg_Batin_Adc > 2.45 && Chg_Current_Adc >= Chg_Current_CV_End) {
				if (1.81 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 2.35) {
					// CV mode L
					cnt_d6++;
					cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = 0;
					if (cnt_d6 >= 10) {
						current_case = caseD6;
						flow_chart_ebike_case(caseD6);
						cnt_d6 = 10;
					}
				}
			} else {
				// Except loop/ Charge_complete mode
				
				next_stage = stage1;
				Initial_Start = 1;
			}
			break;
		case stage7:
			/* Enable Auxilarity DC power in Boost mode */
			// Checking voltage's condition is normal or not 
			if (0.82 > Aux_Bat_Vin_Adc  ||  Aux_Bat_Vin_Adc > 1.2 || Main_Chg_Batin_Adc >= Main_Bat_Input_Max) {
				// Except loop
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (0.82 > Aux_Bat_Vin_Adc) HAL_UART_Transmit(&huart3, (uint8_t *)"0.82>Aux_Bat_Vin_Adc\r\n", strlen("0.82>Aux_Bat_Vin_Adc\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc > 1.2) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc>1.2\r\n", strlen("Aux_Bat_Vin_Adc>1.2\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc >= Main_Bat_Input_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc>=Main_Bat_Input_Max=2.5\r\n", strlen("Main_Chg_Batin_Adc>=Main_Bat_Input_Max=2.5\r\n"), HAL_MAX_DELAY);
#endif
				break;
			}
			cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
			cnt_A21 = cnt_A22 = cnt_A23 = cnt_A24 = 0;
			
			if (Main_Chg_Batin_Adc < 1.75) {
				if (0.82 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.2) {
					cnt_u11++;
					cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
					if (cnt_u11 >= 10) {
						current_case = caseU11;
						flow_chart_ebike_case(caseU11);
						cnt_u11 = 10;
					}
				}
			} else if (Main_Chg_Batin_Adc < 1.92) {
				if (0.82 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.2) {
					cnt_u12++;
					cnt_u11 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
					if (cnt_u12 >= 10) {
						current_case = caseU12;
						flow_chart_ebike_case(caseU12);
						cnt_u12 = 10;
					}
				}
			}	else if (Main_Chg_Batin_Adc < 2.33) {
				if (0.82 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 0.89) {
					cnt_u13++;
					cnt_u11 = cnt_u12 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
					if (cnt_u13 >= 10) {
						current_case = caseU13;
						flow_chart_ebike_case(caseU13);
						cnt_u13 = 10;
					}
				} else if (0.89 < Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 0.9) {
					if (current_case == caseU13 || current_case == caseU14) {
						flow_chart_ebike_case(current_case);
					} else {
						current_case = caseU13;
						flow_chart_ebike_case(current_case);
					}
				} else if (0.9 < Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.2) {
					cnt_u14++;
					cnt_u11 = cnt_u12 = cnt_u13 = cnt_u15 = cnt_u16 = 0;
					if (cnt_u14 >= 10) {
						current_case = caseU14;
						flow_chart_ebike_case(current_case);
						cnt_u14 = 10;
					}
				}
			}	else if (Main_Chg_Batin_Adc < 2.45 && Chg_Current_Adc >= Chg_Current_Target_Low) {
				if (0.82 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.2) {
					cnt_u15++;
					cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u16 = 0;
					if (cnt_u15 >= 10) {
						current_case = caseU15;
						flow_chart_ebike_case(caseU15);
						cnt_u15 = 10;
					}
					init_time_U15 = HAL_GetTick();
				}
			} else if (current_case == caseU15) {
				if (Main_Chg_Batin_Adc <= 2.48 || Chg_Current_Adc < Chg_Current_Target_Low) {
					current_time_U15 = HAL_GetTick();
					if (current_time_U15 - init_time_U15 > 3000) {
						current_case = caseU16;
						flow_chart_ebike_case(caseU16);
					}
				} else {
					current_case = caseU16;
					flow_chart_ebike_case(caseU16);
				}


			} else if (Chg_Current_Adc >= Chg_Current_CV_End) {
				if (0.89 <= Aux_Bat_Vin_Adc && Aux_Bat_Vin_Adc <= 1.2) {
					// CV mode L
					cnt_u16++;
					cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = 0;
					if (cnt_u16 >= 10) {
						current_case = caseU16;
						flow_chart_ebike_case(caseU16);
						cnt_u16 = 10;
					}
				}
			} else {
				// Except loop/ Charge_complete mode
				next_stage = stage1;
				Initial_Start = 1;
			}
			break;
		case stage8:
			cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
			cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
			/* Enable AC power, CAN/RS232 communication */
			MAIN_POWER_ON(LOW);
			AC_CHG_EN(HIGH);
			if (Main_Chg_Batin_Adc < 1.75) {
				cnt_A21++;
				cnt_A22 = cnt_A23 = cnt_A24 = 0;
				if (cnt_A21 >= 10) {
					current_case = caseA21;
					flow_chart_ebike_case(caseA21);
					cnt_A21 = 10;
				}
			} else if (Main_Chg_Batin_Adc < 1.92) {
				cnt_A22++;
				cnt_A21 = cnt_A23 = cnt_A24 = 0;
				if (cnt_A22 >= 10) {
					current_case = caseA22;
					flow_chart_ebike_case(caseA22);
					cnt_A22 = 10;
				}
			} else if (Main_Chg_Batin_Adc < 2.33) {
				cnt_A23++;
				cnt_A21 = cnt_A22 = cnt_A24 = 0;
				if (cnt_A23 >= 10) {
					current_case = caseA23;
					flow_chart_ebike_case(caseA23);
					cnt_A23 = 10;
				}
			} else if (Main_Chg_Batin_Adc <= 2.45) {
				cnt_A24++;
				cnt_A21 = cnt_A22 = cnt_A23 = 0;
				if (cnt_A24 >= 10) {
					current_case = caseA24;
					flow_chart_ebike_case(caseA24);
					cnt_A24 = 10;
				}
				
//			} else if (Main_Chg_Batin_Adc < 1.58) {
//				/* 
//				* When battery reach to 84, you start CV mode
//				* During CV mode, you can encounter two case.
//				* First case, 
//				* During keep external voltage 84V, the charging current will decrease continuously. When current decreasing point 0.3A ? you can stop charging.
//				* But during this operation, there is some of problem, (it is not expected) the current is over 20A, you must stop power providing
//				* Second case
//				* The battery is used for system operation then battery voltage can be decreased. 
//				* But there is little voltage drop, you do not need CC mode again,
//				* If the battery below 79V, charge operated as CC,mode again
//				* This means, once charger operated as CV mode. You keep CV during battery is over 79V
//				*/
//				if (current_case == caseA23 || current_case == caseA24)
//					flow_chart_ebike_case(current_case);
//				else {
//					current_case = caseA23;
//					flow_chart_ebike_case(caseA23);
//				}					
//			} else if (Main_Chg_Batin_Adc < 1.65) {
//				current_case = caseA24;
//				flow_chart_ebike_case(caseA24);
//			} 
			
			} else if (Main_Chg_Batin_Adc >= Main_Bat_Input_Max) {
				// Except loop/ Charge_complete mode
				next_stage = stage1;
				Initial_Start = 1;
			} else {
				// Except loop/ Charge_complete mode
				next_stage = stage1;
				Initial_Start = 1;
			}

			break;
		case stage9:
			cnt_d1 = cnt_d2 = cnt_d3 = cnt_d4 = cnt_d5 = cnt_d6 = 0;
			cnt_u11 = cnt_u12 = cnt_u13 = cnt_u14 = cnt_u15 = cnt_u16 = 0;
			/* Enable AC power, not communication */
			MAIN_POWER_ON(LOW);
			AC_CHG_EN(HIGH);
			flow_chart_ebike_case(caseA31);
			break;
		case stageA31:
			//	Check whether or not communicate with AC power through CAN/RS485/RS232
			AC_Respond_flag = check_comunication_AC_power();
			if (AC_Respond_flag == 1) {
				turn_on_AC_power();
				set_ACpower_voltage(60);
				if (VoltageCheckAC() == bad) {
					next_stage = stage0;
				} else {
					// VoltageCheckAC() == good
					next_stage = stage3;
				}
			} else {
				// AC_Respond_flag == 0
				next_stage = stageA34;
			}
			break;
		case stageA34:
			// Goto here when can not communicate with AC power
			// Check whether or not communicate with Aux Battery power through CAN/RS485/RS232
			AuxBattery_Respond_flag = check_comunication_AuxBattery_power();
			if (AuxBattery_Respond_flag == 1) {
					turn_AuxBatterySwitch_on();
					if (VoltageCheckAuxBattery() == bad) {
						next_stage = stage0;
					} else {
						// VoltageCheckAuxBattery() == good
						next_stage = stage3;
					}
			} else {
				// AuxBattery_Respond_flag == 0
				next_stage = stage0;
			}
			break;
		default:
			next_stage = stage0;
			break;
	}		
}

void flow_chart_ebike_case(int number) {
	switch (number) {
		case Input_Power_on:
			break;
		
/************************ Case of Stage 6 ************************/
		case caseD1:
			// Main_Bat_Wake_DC() - Buck mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 5;
			if (Chg_Current_Adc >= Chg_Current_Lim_Max || Aux_Bat_Vin_Adc < Aux_Bat_Min_28) {
				// Except loop
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD1\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
#endif
			}
//			DC_Pwm_S();		
			break;
		case caseD2:
			// Trickle_Charging_DC() - low current charging - Buck mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 5;
			if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28 || Chg_Current_Adc > Chg_Current_Lim_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD2\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
#endif
			}
//			DC_Pwm_ST();
			break;
		case caseD3:
			// LP_Charge_Mode - Buck mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(HIGH);
			Bat_Wakeup_Time_count = 1;
//			Chg_Target_Vout = Main_Bat_CV;
			if (Chg_Current_Adc > Chg_Current_Lim_Max || Aux_Bat_Vin_Adc < Aux_Bat_Min_28 || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD3\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);		
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);	
#endif
			}
//			DC_Pwm_ST();					
			break;
		case caseD4:
			// Normal_CC_Mode - Buck mode
			if (Phase_Mode == 1) {
				Phase_Mode = 2;
				HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
				
				// Set BUCK TOP2 180 degree
				__HAL_TIM_SET_COUNTER(&htim17, 799);
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);			
				HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim17,TIM_CHANNEL_1);					
			}
			
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(HIGH);
			ANTI_REVERSE(HIGH);
			Buck_Top2_Phase_Adj = 1;					//  (180 degree)
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV; 		// = 1.59
			if (Chg_Current_Adc > Chg_Current_Lim_Max ||  Aux_Bat_Vin_Adc < Aux_Bat_Min_28 || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD4\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);		
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);	
#endif
			}
			//			CV_mode()
			
			break;
		case caseD5:
			// CV_Mode_H - Buck mode
			if (Phase_Mode == 1) {
				Phase_Mode = 2;
				HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim15,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim17,TIM_CHANNEL_1);
				
				// Set BUCK TOP2 180 degree
				__HAL_TIM_SET_COUNTER(&htim17, 799);
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);	
				HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim17,TIM_CHANNEL_1);		
			}
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(HIGH);
			ANTI_REVERSE(HIGH);
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV;				// Chg_Target_Vout = 1.59V
			if (Chg_Current_Adc > Chg_Current_Lim_Max || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max || \
						Aux_Bat_Vin_Adc < Aux_Bat_Min_28 ) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG							
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD5\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);		
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);	
#endif
			}
			break;
		case caseD6:
			// CV_Mode_L - Buck mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV;
			if (Chg_Current_Adc > Chg_Current_Lim_Max || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max || \
						Aux_Bat_Vin_Adc < Aux_Bat_Min_28) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.6-caseD6\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_28) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_28=1.64\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);		
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);	
#endif		
			}
			if (Main_Chg_Batin_Adc > 2.48) {
				next_stage = stage1;
				current_case = Input_Power_on;
				Initial_Start = 1;
				break;
			}
			if (2.45 < Main_Chg_Batin_Adc && Main_Chg_Batin_Adc < 2.48 && Chg_Current_Adc < Chg_Current_CV_End) {
				next_stage = stage1;
				current_case = Input_Power_on;
				Initial_Start = 1;
				break;
			}
			if (Main_Chg_Batin_Adc <= 2.48 && Chg_Current_Adc < Chg_Current_CV_End && flag1_D6 == 0) {
				flag1_D6 = 1;
				flag2_D6 = 0;
				init_time_D6 = HAL_GetTick();
			} else if (2.45 < Main_Chg_Batin_Adc && Main_Chg_Batin_Adc < 2.48 && flag2_D6 == 0) {
				flag1_D6 = 0;
				flag2_D6 = 1;
				init_time_D6 = HAL_GetTick();
			} else if (Main_Chg_Batin_Adc <= 2.45) {
				flag1_D6 = 0;
				flag2_D6 = 0;
			}
			if (flag1_D6 == 1 || flag2_D6 == 1) {
				current_time_D6 = HAL_GetTick();
				if (current_time_D6 - init_time_D6 > 3000) {
					next_stage = stage1;
					current_case = Input_Power_on;
					Initial_Start = 1;
				}
			}
			break;
/********************** End Case of Stage 6 **********************/
		
/************************ Case of Stage 7 ************************/
		case caseU11:
			// Main_Bat_Wake_DC() - Boost mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 5;
			if (Chg_Current_Adc >= Chg_Current_Lim_Max || Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || \
						Main_Chg_Batin_Adc >= Main_Bat_Input_Max) {
				// Except loop
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU11\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc > Main_Bat_Input_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc>Main_Bat_Input_Max=2.5\r\n", strlen("Main_Chg_Batin_Adc>Main_Bat_Input_Max=2.5\r\n"), HAL_MAX_DELAY);
#endif
			}
//			DC_Pwm_S();		
			break;
		case caseU12:
			// Trickle_Charging_DC() - low current charging - Boost mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 5;
			if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || Chg_Current_Adc > Chg_Current_Lim_Max || \
					Main_Chg_Batin_Adc > Main_Bat_Input_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU12\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc > Main_Bat_Input_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc>Main_Bat_Input_Max=2.5\r\n", strlen("Main_Chg_Batin_Adc>Main_Bat_Input_Max=2.5\r\n"), HAL_MAX_DELAY);
#endif
			}
//			DC_Pwm_ST();
			break;
		case caseU13:
			// LP_Charge_Mode - Boost mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(HIGH);
			Bat_Wakeup_Time_count = 1;
//			Chg_Target_Vout = Main_Bat_CV;
			if (Chg_Current_Adc > Chg_Current_Lim_Max || Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU13\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
#endif
			}
//			DC_Pwm_ST();					
			break;
		case caseU14:
			// Normal_CC_Mode - Boost mode
			if (Phase_Mode == 1) {
				Phase_Mode = 2;
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
				
				// Set BOOT TOP2 180 degree
				__HAL_TIM_SET_COUNTER(&htim1, 799);
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);				
			}
			
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(HIGH);
			ANTI_REVERSE(HIGH);
			Boost_Top2_Phase_Adj = 1;					//  (180 degree)
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV; 		// = 1.59
			if (Chg_Current_Adc > Chg_Current_Lim_Max ||  Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU14\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
#endif
			}
			//			CV_mode()
			
			break;
		case caseU15:
			// CV_Mode_H - Boost mode
			if (Phase_Mode == 1) {
				Phase_Mode = 2;
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
				
				// Set BOOT TOP2 180 degree
				__HAL_TIM_SET_COUNTER(&htim1, 799);
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);			
			}

			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(HIGH);
			ANTI_REVERSE(HIGH);
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV;				// Chg_Target_Vout = 1.59V
			if (Chg_Current_Adc > Chg_Current_Lim_Max || Chg_Current_Adc < Chg_Current_Target_Low || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max || \
						Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max || Main_Chg_Batin_Adc < Main_Bat_CV_CC) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG							
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU15\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Chg_Current_Adc < Chg_Current_Target_Low) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc<Chg_Current_Target_Low=0.12\r\n", strlen("Chg_Current_Adc<Chg_Current_Target_Low=0.12\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);			
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc>Main_Bat_CV_CC_Max=2.45\r\n", strlen("Main_Chg_Batin_Adc>Main_Bat_CV_CC_Max=2.45\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc < Main_Bat_CV_CC) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc<Main_Bat_CV_CC=2.27\r\n", strlen("Main_Chg_Batin_Adc<Main_Bat_CV_CC=2.27\r\n"), HAL_MAX_DELAY);
#endif
			}
			break;
		case caseU16:
			// CV_Mode_L - Boost mode
			Phase_Mode = 1;
			AC_CHG_EN(LOW);
			MAIN_POWER_ON(HIGH);
			BUCK_BOOST_OFF_P1(HIGH);
			BUCK_BOOST_OFF_P2(LOW);
			ANTI_REVERSE(LOW);
			Bat_Wakeup_Time_count = 1;
			Chg_Target_Vout = Main_Bat_CV;
			if (Chg_Current_Adc > Chg_Current_Lim_Max || Chg_Current_Adc < Chg_Current_CV_End || \
					Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max || Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max || \
						Aux_Bat_Vin_Adc < Aux_Bat_Min_14 || Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max || Main_Chg_Batin_Adc < Main_Bat_CV_CC) {
				software_protection_flag = 1;
				next_stage = stage1;
				current_case = Input_Power_on;
#ifdef DEBUG
				memset(buffer_debug, 0, sizeof(buffer_debug));
				snprintf(buffer_debug, 80, "[ERROR]: %s, %d, stage.7-caseU16\r\n", __func__, __LINE__);
				HAL_UART_Transmit(&huart3, (uint8_t *)buffer_debug, strlen(buffer_debug), HAL_MAX_DELAY);
				
				if (Chg_Current_Adc > Chg_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n", strlen("Chg_Current_Adc>Chg_Current_Lim_Max=2.4\r\n"), HAL_MAX_DELAY);
				if (Chg_Current_Adc < Chg_Current_CV_End) HAL_UART_Transmit(&huart3, (uint8_t *)"Chg_Current_Adc<Chg_Current_CV_End=0.03\r\n", strlen("Chg_Current_Adc<Chg_Current_CV_End=0.03\r\n"), HAL_MAX_DELAY);
				if (Phase1_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase1_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);
				if (Phase2_Cur_Adc > Chg_Phase_Current_Lim_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n", strlen("Phase2_Cur_Adc>Chg_Phase_Current_Lim_Max=3.2\r\n"), HAL_MAX_DELAY);			
				if (Aux_Bat_Vin_Adc < Aux_Bat_Min_14) HAL_UART_Transmit(&huart3, (uint8_t *)"Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n", strlen("Aux_Bat_Vin_Adc<Aux_Bat_Min_14=0.82\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc>Main_Bat_CV_CC_Max=2.45\r\n", strlen("Main_Chg_Batin_Adc>Main_Bat_CV_CC_Max=2.45\r\n"), HAL_MAX_DELAY);
				if (Main_Chg_Batin_Adc < Main_Bat_CV_CC) HAL_UART_Transmit(&huart3, (uint8_t *)"Main_Chg_Batin_Adc<Main_Bat_CV_CC=2.27\r\n", strlen("Main_Chg_Batin_Adc<Main_Bat_CV_CC=2.27\r\n"), HAL_MAX_DELAY);

#endif		
			}
			if (Main_Chg_Batin_Adc > 2.48) {
				next_stage = stage1;
				current_case = Input_Power_on;
				Initial_Start = 1;
				break;
			}
			if (2.45 < Main_Chg_Batin_Adc && Main_Chg_Batin_Adc < 2.48 && Chg_Current_Adc < Chg_Current_CV_End) {
				next_stage = stage1;
				current_case = Input_Power_on;
				Initial_Start = 1;
				break;
			}
			
			if (Main_Chg_Batin_Adc <= 2.48 && Chg_Current_Adc < Chg_Current_CV_End && flag1_U16 == 0) {
				flag1_U16 = 1;
				flag2_U16 = 0;
				init_time_U16 = HAL_GetTick();
			} else if (2.45 < Main_Chg_Batin_Adc && Main_Chg_Batin_Adc < 2.48 && flag2_U16 == 0) {
				flag1_U16 = 0;
				flag2_U16 = 1;
				init_time_U16 = HAL_GetTick();
			} else if (Main_Chg_Batin_Adc <= 2.45) {
				flag1_U16 = 0;
				flag2_U16 = 0;
			}
			if (flag1_U16 == 1 || flag2_U16 == 1) {
				current_time_U16 = HAL_GetTick();
				if (current_time_U16 - init_time_U16 > 3000) {
					next_stage = stage1;
					current_case = Input_Power_on;
					Initial_Start = 1;
				}
			}
			break;
/********************** End Case of Stage 7 **********************/
		
/************************ Case of Stage 8 ************************/
		case caseA21:
			// Main_Bat_Wake_AC()
			if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) {
				software_protection_flag = 1;
				// EV3 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc > Chg_Current_Lim_Max)	{			// I think this should be checked
				software_protection_flag = 1;
				EV6 = 1;
				next_stage = stage1;
				break;
			}
			
			// EV3 = 0;				
			EV6 = 0;
			if (Chg_Current_Adc <= Chg_Current_Target_Low && Main_Chg_Batin_Adc <= Main_Bat_Input_Max) {
				Bat_Wakeup_Time_count = 5;
				Step_Time_Int = Feed_Time_Basic * Step_Time_Count * Bat_Wakeup_Time_count;
				// CAN/RS485/RS232 communicate to increase voltage
				set_ACpower_current(Chg_Current_Target_Low);
			} else {
				set_ACpower_current(0);
			}
			break;
		case caseA22:
			// Trickle_Charging_AC()
			if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) {
				software_protection_flag = 1;
				// EV3 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc > Chg_Current_Lim_Max)	{			// I think this should be checked
				software_protection_flag = 1;
				EV6 = 1;
				next_stage = stage1;
				break;
			}
			
			// EV3 = 0;				
			EV6 = 0;			
			if (Chg_Current_Adc <= Chg_Current_Target_Low) {
				Bat_Wakeup_Time_count = 5;
				Step_Time_Int = Feed_Time_Basic * Step_Time_Count * Bat_Wakeup_Time_count;
				// CAN/RS485/RS232 communicate to increase voltage
				set_ACpower_current(Chg_Current_Target_Low);
			} else {
				set_ACpower_current(0);
			}
			break;
		case caseA23:
			// Normal_CC_Mode_AC
			if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) {
				software_protection_flag = 1;
				// EV3 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc > Chg_Current_Lim_Max)	{			// I think this should be checked
				software_protection_flag = 1;
				EV6 = 1;
				next_stage = stage1;
				break;
			}
			
			// EV3 = 0;				
			EV6 = 0;
			if (Chg_Current_Adc <= 1.05 * 1.2) {
				set_ACpower_current(15);
			} else {
				set_ACpower_current(0);
			}
			break;
		case caseA24:
			// CV_Mode_AC
			if (Main_Chg_Batin_Adc > Main_Bat_Input_Max) {
				software_protection_flag = 1;
				// EV3 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc > Chg_Current_Lim_Max) {			// I think this should be checked
				software_protection_flag = 1;
				EV6 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc < Chg_Current_CV_End) {
				
			}
			
			// EV3 = 0;				
			EV6 = 0;
			
			break;
/********************** End Case of Stage 8 **********************/
		
/************************ Case of Stage 9 ************************/
		case caseA31:
			// Charger_AC_direct
			if (Main_Chg_Batin_Adc > Main_Bat_CV_CC_Max) {
				software_protection_flag = 1;
				// EV3 = 1;
				next_stage = stage1;
				break;
			}
			if (Chg_Current_Adc > Chg_Current_Lim_Max) {
				software_protection_flag = 1;
				EV6 = 1;
				next_stage = stage1;
				break;
			}
			// EV3 = 0;				
			EV6 = 0;
			break;
/********************** End Case of Stage 9 **********************/
		default:
			break;
	}
}


void SetDutyCircle(float dutyCircle, TIM_HandleTypeDef *htim) {
	// Calculate Pulse
	int target_Pulse = 0;
	double PulseBeforeRound = 0.0;
	
	// With fPWM = 100KHz, Prescale = 0, fTimer = 16MHz => Period = 159
	// Pulse must be interger => when insert Duty circle it will not Precise because Pulse is rounded
	PulseBeforeRound = PWM_COUNTER_MAX * (float)dutyCircle / 100;
	target_Pulse = ROUND(PulseBeforeRound);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, target_Pulse);
}
void SetDutyCircle_BUCK1(float dutyCircle, uint8_t force)
{
	// Calculate Pulse
//	static int current_Pulse_BUCK1 = 0;
	int target_Pulse = 0;
	double PulseBeforeRound = 0.0;
	
	// With fPWM = 100KHz, Prescale = 0, fTimer = 16M0Hz => Period = 1599
	// target_Pulse must be interger => when insert Duty circle it will not Precise because Pulse is rounded
	PulseBeforeRound = PWM_COUNTER_MAX * dutyCircle / 100.0f;
	target_Pulse = ROUND(PulseBeforeRound);
	if (force) {
		PWM_counter_BUCK1 = target_Pulse;
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
	} else {
		if ((PWM_counter_BUCK1 - PWM_STEPSIZE < target_Pulse) && (target_Pulse < PWM_counter_BUCK1 + PWM_STEPSIZE)) {
			PWM_counter_BUCK1 = target_Pulse;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		} else if (PWM_counter_BUCK1 < target_Pulse) {
			PWM_counter_BUCK1 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		} else {
			PWM_counter_BUCK1 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		}
	}
}

void SetDutyCircle_BUCK2(float dutyCircle, uint8_t force)
{
	// Calculate Pulse
//	static int PWM_counter_BUCK2 = 0;
	int target_Pulse = 0;
	double PulseBeforeRound = 0.0;
	
	// With fPWM = 100KHz, Prescale = 0, fTimer = 16MHz => Period = 159
	// Pulse must be interger => when insert Duty circle it will not Precise because Pulse is rounded
	PulseBeforeRound = PWM_COUNTER_MAX * (float)dutyCircle / 100;
	target_Pulse = ROUND(PulseBeforeRound);
	if (force) {
		PWM_counter_BUCK2 = target_Pulse;
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
	} else {
		if ((PWM_counter_BUCK2 - PWM_STEPSIZE < target_Pulse) && (target_Pulse < PWM_counter_BUCK2 + PWM_STEPSIZE)) {
			PWM_counter_BUCK2 = target_Pulse;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		} else if (PWM_counter_BUCK2 < target_Pulse) {
			PWM_counter_BUCK2 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		} else {
			PWM_counter_BUCK2 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		}
	}
}

void SetDutyCircle_BOOST1(float dutyCircle, uint8_t force)
{
	// Calculate Pulse
	int target_Pulse = 0;
	double PulseBeforeRound = 0.0;
	
	// With fPWM = 100KHz, Prescale = 0, fTimer = 16MHz => Period = 159
	// Pulse must be interger => when insert Duty circle it will not Precise because Pulse is rounded
	PulseBeforeRound = PWM_COUNTER_MAX * (float)dutyCircle / 100;
	target_Pulse = ROUND(PulseBeforeRound);
	
	if (target_Pulse < PWM_COUNTER_MIN) target_Pulse = PWM_COUNTER_MIN;
	if (target_Pulse > PWM_COUNTER_BOOST_MAX) target_Pulse = PWM_COUNTER_BOOST_MAX;
	
	if (force) {
		PWM_counter_BOOST1 = target_Pulse;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
	} else {
		if ((PWM_counter_BOOST1 - PWM_STEPSIZE < target_Pulse) && (target_Pulse < PWM_counter_BOOST1 + PWM_STEPSIZE)) {
			PWM_counter_BOOST1 = target_Pulse;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		} else if (PWM_counter_BOOST1 < target_Pulse) {
			PWM_counter_BOOST1 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		} else {
			PWM_counter_BOOST1 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		}
	}
}

void SetDutyCircle_BOOST2(float dutyCircle, uint8_t force)
{
	// Calculate Pulse
//	static int PWM_counter_BOOST2 = 0;
	int target_Pulse = 0;
	double PulseBeforeRound = 0.0;
	
	// With fPWM = 100KHz, Prescale = 0, fTimer = 16MHz => Period = 159
	// Pulse must be interger => when insert Duty circle it will not Precise because Pulse is rounded
	PulseBeforeRound = PWM_COUNTER_MAX * (float)dutyCircle / 100;
	target_Pulse = ROUND(PulseBeforeRound);

	if (target_Pulse < PWM_COUNTER_MIN) target_Pulse = PWM_COUNTER_MIN;
	if (target_Pulse > PWM_COUNTER_BOOST_MAX) target_Pulse = PWM_COUNTER_BOOST_MAX;
	
	if (force) {
		PWM_counter_BOOST2 = target_Pulse;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
	} else {
		if ((PWM_counter_BOOST2 - PWM_STEPSIZE < target_Pulse) && (target_Pulse < PWM_counter_BOOST2 + PWM_STEPSIZE)) {
			PWM_counter_BOOST2 = target_Pulse;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		} else if (PWM_counter_BOOST2 < target_Pulse) {
			PWM_counter_BOOST2 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		} else {
			PWM_counter_BOOST2 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		}
	}
}

void SetPWM_BUCK1(int PWM_target, uint8_t force)
{
	// Calculate PWM counter
	if (PWM_target < PWM_COUNTER_MIN) PWM_target = PWM_COUNTER_MIN;
	if (PWM_target > PWM_COUNTER_MAX) PWM_target = PWM_COUNTER_MAX;
	
	if (force) {
		PWM_counter_BUCK1 = PWM_target;
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
	} else {
		if ((PWM_counter_BUCK1 - PWM_STEPSIZE < PWM_target) && (PWM_target < PWM_counter_BUCK1 + PWM_STEPSIZE)) {
			PWM_counter_BUCK1 = PWM_target;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		} else if (PWM_counter_BUCK1 < PWM_target) {
			PWM_counter_BUCK1 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		} else {
			PWM_counter_BUCK1 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_counter_BUCK1);
		}
	}
}

void SetPWM_BUCK2(int PWM_target, uint8_t force)
{
	// Calculate Pulse
	if (PWM_target < PWM_COUNTER_MIN) PWM_target = PWM_COUNTER_MIN;
	if (PWM_target > PWM_COUNTER_MAX) PWM_target = PWM_COUNTER_MAX;

	if (force) {
		PWM_counter_BUCK2 = PWM_target;
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
	} else {
		if ((PWM_counter_BUCK2 - PWM_STEPSIZE < PWM_target ) && (PWM_target < PWM_counter_BUCK2 + PWM_STEPSIZE)) {
			PWM_counter_BUCK2 = PWM_target;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		} else if (PWM_counter_BUCK2 < PWM_target) {
			PWM_counter_BUCK2 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		} else {
			PWM_counter_BUCK2 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PWM_counter_BUCK2);
		}
	}
}

void SetPWM_BOOST1(int PWM_target, uint8_t force)
{
	// Calculate PWM counter
	if (PWM_target < PWM_COUNTER_MIN) PWM_target = PWM_COUNTER_MIN;
	if (PWM_target > PWM_COUNTER_BOOST_MAX) PWM_target = PWM_COUNTER_BOOST_MAX;
	
	if (force) {
		PWM_counter_BOOST1 = PWM_target;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
	} else {
		if ((PWM_counter_BOOST1 - PWM_STEPSIZE < PWM_target) && (PWM_target < PWM_counter_BOOST1 + PWM_STEPSIZE)) {
			PWM_counter_BOOST1 = PWM_target;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		} else if (PWM_counter_BOOST1 < PWM_target) {
			PWM_counter_BOOST1 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		} else {
			PWM_counter_BOOST1 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_counter_BOOST1);
		}
	}
}

void SetPWM_BOOST2(int PWM_target, uint8_t force)
{
	// Calculate PWM counter
	if (PWM_target < PWM_COUNTER_MIN) PWM_target = PWM_COUNTER_MIN;
	if (PWM_target > PWM_COUNTER_BOOST_MAX) PWM_target = PWM_COUNTER_BOOST_MAX;
	
	if (force) {
		PWM_counter_BOOST2 = PWM_target;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
	} else {
		if ((PWM_counter_BOOST2 - PWM_STEPSIZE < PWM_target) && (PWM_target < PWM_counter_BOOST2 + PWM_STEPSIZE)) {
			PWM_counter_BOOST2 = PWM_target;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		} else if (PWM_counter_BOOST2 < PWM_target) {
			PWM_counter_BOOST2 += PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		} else {
			PWM_counter_BOOST2 -= PWM_STEPSIZE;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_counter_BOOST2);
		}
	}
}

uint8_t DCVoltageCheck()
{
	if (Vcheck_12V < 2.3 || Vcheck_12V > 2.75)
	{
		return bad;
	}
	if (Vcheck_5V < 0.9 || Vcheck_5V > 1.17)
	{
		return bad;
	}
	if (ACchg_Bat_Vin_Adc > 2.5)
	{
		EV2 = 1;
		return bad;
	}
	if (Aux_Bat_Vin_Adc > 2.5)
	{
		EV2 = 1;
		return bad;
	}
	EV2 = 0;
	return good;
}

uint8_t VoltageCheckAC()
{
	if (Vcheck_12V < 2.16 || Vcheck_12V > 2.806)
	{
		return bad;
	}
	if (Vcheck_5V < 0.9 || Vcheck_5V > 1.17)
	{
		return bad;
	}
	return good;
}

uint8_t VoltageCheckAuxBattery()
{
	if (Vcheck_12V < 2.16 || Vcheck_12V > 2.806)
	{
		return bad;
	}
	if (Vcheck_5V < 0.9 || Vcheck_5V > 1.17)
	{
		return bad;
	}
	return good;
}

uint8_t check_comunication_AC_power()
{
	// TODO
	// hardcode return 0;
	return 0;
}
void turn_on_AC_power() {
	// TODO
}

void turn_AuxBatterySwitch_on() {
	// TODO
}

uint8_t check_comunication_AuxBattery_power()
{
	// TODO
	// hardcode return 0;
	return 0;
}

void set_ACpower_current(float Setpoint_current)
{
	// TODO
}
void set_ACpower_voltage(float Setpoint_voltage)
{
	// TODO
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->previous_error = 0.0f;
	pid->integral = 0.0f;
}

// Caculate duty cycle PID
float output = 0;
float PID_Compute(PID_Controller *pid, float setpoint, float current_value, float dt)
{
	float error = setpoint - current_value;									// Error
	pid->integral += error * dt;  													// Integral
	float derivative = (error - pid->previous_error) / dt;  // Derivative

	// Calculate duty cycle
//	float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
	output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

	// Limit duty cycle (0-100%)
	if (output > 100.0f) output = 100.0f;
	if (output < 0.0f) output = 0.0f;

	// Update error 
	pid->previous_error = error;

	return output; 
}

// Caculate PWM adj for CC mode
int PID_Compute_PWM_adj(PID_Controller *pid, float setpoint, float current_value, float dt)
{
	float error = setpoint - current_value;									// Error
	pid->integral += error * dt;  													// Integral
	float derivative = (error - pid->previous_error) / dt;  // Derivative

	// Calculate duty cycle
	int output = (int)((pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative));

	// Update error
	pid->previous_error = error;

	return output; 
}