#ifndef __CHARGER_H
#define __CHARGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "process_data.h"

#define PWM_COUNTER_MAX  	1599							// TIMx->ARR
#define PWM_COUNTER_BOOST_MAX 800
#define PWM_COUNTER_MIN  	0
#define	PWM_STEPSIZE			1

#define AC_CHG_EN(GPIO_PIN_STATE) 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_STATE)
#define MAIN_POWER_ON(GPIO_PIN_STATE) 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_STATE)
#define	BUCK_BOOST_OFF_P1(GPIO_PIN_STATE) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_STATE)
#define	ANTI_REVERSE(GPIO_PIN_STATE) 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_STATE)
#define	BUCK_BOOST_OFF_P2(GPIO_PIN_STATE) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_STATE)
#define READ_HARD_PROTECTION							HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)

extern int next_stage;
extern int current_stage;
extern int previous_stage ;
extern int current_case;
extern int previous_case;

extern char *List_Stage[];
extern char *List_Case[];

enum Index_Stage {
	stage0 = 0,
	stage1,
	stage2,
	stage3,
	stage4,
	stage5,
	stage6,
	stage7,
	stage8,
	stage9,
	stageA1,
	stageA31,
	stageA32,
	stageA33,
	stageA34,
	laststage
};
enum Index_Case {
	Input_Power_on = 0,
	// Stage 6 in promode
	caseD1,
	caseD2,
	caseD3,
	caseD4,
	caseD5,
	caseD6,
	caseD7,				// Reserved
	caseD8,				// Reserved
	caseD9,				// Reserved
	caseD10,			// Reserved
	// Stage 7 in promode
	caseU11,
	caseU12,
	caseU13,
	caseU14,
	caseU15,
	caseU16,
	caseU17,			// Reserved
	caseU18,			// Reserved
	caseU19,			// Reserved
	caseU20,			// Reserved
	// Stage 8 in promode
	caseA21,
	caseA22,
	caseA23,
	caseA24,
	// Stage 9 in promode
	caseA31,
	lastcase
};

typedef enum {
    PWM_decrease = -1,
    PWM_hold = 0,
    PWM_increase = 1
} PWM_Direction;

typedef struct {
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
} PID_Controller;

extern PID_Controller pid_caseD1, pid_caseD2, pid_caseD3, \
					pid_caseD4, pid_caseD4_Phase1_Cur, pid_caseD4_Phase2_Cur, \
					pid_caseD5, pid_caseD5_Phase1_Cur, pid_caseD5_Phase2_Cur, \
					pid_caseD6;

extern PID_Controller pid_caseU11, pid_caseU12, pid_caseU13, \
					pid_caseU14, pid_caseU14_Phase1_Cur, pid_caseU14_Phase2_Cur, \
					pid_caseU15, pid_caseU15_Phase1_Cur, pid_caseU15_Phase2_Cur, \
					pid_caseU16;

extern uint8_t flag1_D6, flag2_D6;
extern uint8_t flag1_U16, flag2_U16;
extern int hardware_protection_flag, software_protection_flag;

extern uint16_t u16_ADC1_Val[2];
extern uint16_t u16_ADC2_Val[9];
extern uint16_t u16_ADC4_Val[2];
extern uint16_t u16_ADC1_Val_average[2];
extern uint16_t u16_ADC2_Val_average[9];
extern uint16_t u16_ADC4_Val_average[2];
extern float Vcheck_12V;
extern float Vcheck_5V;

// CHG_CURRENT - 
extern float Chg_Current_Adc;
extern float Chg_High_Current_Adc;
extern float Phase1_Cur_Adc;
extern float Phase2_Cur_Adc;
extern float Chg_Current_Target_Low;
extern float Chg_Current_Target_Mid;
extern float Chg_Current_Lim_Max;
extern float Chg_CC_Current;
extern float Chg_Phase_Current;	
extern float Chg_Phase_Current_CV_End;
extern float Chg_Current_CV_End;
extern float Chg_Phase_Current_Max;

// MODE_SELECTION 
extern uint8_t Mode_Define_Power;
extern uint8_t Mode_Define_HD;
extern uint8_t Buck_Mode;
extern uint8_t Boost_Mode;
// MAIN_CHG_BATVIN
extern float Main_Chg_Batin_Adc;
extern float Chg_Target_Vout;
extern float Main_Bat_Input_Max;
extern float Main_Bat_CV;
// ACCHG_BAT_VIN
extern float ACchg_Bat_Vin_Adc;
// AUX_BAT_VIN
extern float Aux_Bat_Vin_Adc;

extern float Feed_Time_Basic;

extern int Pwm_Basic_Step;
extern int Pwm_Step_Count;
extern int Pwm_Step_Up_Down_Basic;

extern int Buck_Pwm_Top_Start;
extern int Pwm_Top_Diff_Adj;
extern int Pwm_Top_Diff_Step_count;

extern int Step_Time_Count;
extern int Step_Time_Int;
extern int Bat_Wakeup_Time_count;

extern int Pwm_Step_Up_Down;
// BUCK_TOP1
extern int Buck_Pwm_Top1;				// = Buck_Pwm_Top_Start +/-Pwm_Step_Up_Down* No( increase or decreae )+/-Buck_Pwm_Top_Diff_Adj
extern int Pwm_Step_Up_Down_Phase1_Cur;
extern int Buck_Pwm_Top_Diff_Adj;		// Buck_Pwm_Top_Diff_Adj = Pwm_Top_Diff_Adj* Pwm_Top_Diff_Step_Count
// BUCK_BOT1
extern int Buck_Pwm_Bot1;

// BUCK_TOP2
extern int Buck_Pwm_Top2;						// = Buck_Pwm_Top_Start +/-Pwm_Step_Up_Down* No( increase or decreae )+/-Buck_Pwm_Top_Diff_Adj * zero( 0) 
																		// ( up down decision by target current compare)
extern int Pwm_Step_Up_Down_Phase2_Cur;
extern int Buck_Top2_Phase_Adj;		// 0 = same phase
																			// 1 = 180 degree difference phase between buck 1/ buck 2 ??
extern int Buck_Pwm_Bot2;

extern int Boost_Pwm_Top_Start;
extern int Boost_Pwm_Top_Diff_Adj;
// BOOST_BOT1
extern int Boost_Pwm_Bot1;
extern int Boost_Pwm_Bot1_Diff_Adj;
extern int Boost_Pwm_Bot_Max;
// BOOST_TOP1
extern int Boost_Pwm_Top1;
// BOOST_TOP2
extern int Boost_Pwm_Top2;
extern int Boost_Top2_Phase_Adj;
// BOOST_BOT2
extern int Boost_Pwm_Bot2;

extern float temp_NTC1, temp_NTC2, temp_NTC3, temp_NTC4;
extern uint8_t temp_NTCx_status;
	
extern float Dutycycle_Setpoint;

extern float duty_cycle_BUCK1;
extern float duty_cycle_BUCK2;
extern float duty_cycle_BOOST1;
extern float duty_cycle_BOOST2;

extern int PWM_counter_BUCK1;
extern int PWM_counter_BUCK2;
extern int PWM_counter_BOOST1;
extern int PWM_counter_BOOST2;

extern float real_duty_cycle_BUCK1;
extern float real_duty_cycle_BUCK2;
extern float real_duty_cycle_BOOST1;
extern float real_duty_cycle_BOOST2;

void flow_chart_ebike_stage();
void flow_chart_ebike_case(int number);
void SetDutyCircle(float dutyCircle, TIM_HandleTypeDef *htim);
void SetDutyCircle_BUCK1(float dutyCircle, uint8_t force);
void SetDutyCircle_BUCK2(float dutyCircle, uint8_t force);
void SetDutyCircle_BOOST1(float dutyCircle, uint8_t force);
void SetDutyCircle_BOOST2(float dutyCircle, uint8_t force);

void SetPWM_BUCK1(int PWM_target, uint8_t force);
void SetPWM_BUCK2(int PWM_target, uint8_t force);
void SetPWM_BOOST1(int PWM_target, uint8_t force);
void SetPWM_BOOST2(int PWM_target, uint8_t force);
uint8_t DCVoltageCheck();
uint8_t check_comunication_AC_power();
uint8_t check_comunication_AuxBattery_power();
uint8_t VoltageCheckAC();
uint8_t VoltageCheckAuxBattery();
void turn_on_AC_power();
void set_AC_power_60V();
void set_ACpower_current(float Setpoint_current);
void set_ACpower_voltage(float Setpoint_voltage);

void SetDutyCircleTIM15(float dutyCircle);

void turn_AuxBatterySwitch_on();

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller *pid, float setpoint, float current_value, float dt);
int PID_Compute_PWM_adj(PID_Controller *pid, float setpoint, float current_value, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __CHARGER_H */
