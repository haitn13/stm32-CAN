#include "charger.h"
#include "process_data.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "fdcan.h"

#define MAX_MESSAGE_ID 12

float VI = 0;
float VO = 0;
float PH1_C = 0;
float PH2_C = 0;
float TOT_C = 0;
float TH1 = 0;
float TH2 = 0;
float TH3 = 0;
float TH4 = 0;
float TH5 = 0;
uint8_t EV1 = 0;			// Hardware protection event generation
uint8_t EV2 = 0;			// input over voltage protection event generation 
uint8_t EV3 = 0;			// output over voltage event generation 
uint8_t EV4 = 0;			// Ph1 over current protection event generatio
uint8_t EV5 = 0;			// Ph2 over current protection event generation
uint8_t EV6 = 0;			// Tot over current protection event generation
uint8_t EV7 = 0;			// reserve current protection event generation
uint8_t EV8 = 0;			// over temp event generation
uint8_t EV9 = 0;			// reserve 
uint8_t EV10 = 0;			// reserve 
uint8_t EV11 = 0;			// reserve 
uint8_t EV12 = 0;			// reserve 
uint8_t EV13 = 0;			// reserve 
uint8_t EV14 = 0;			// reserve 
uint8_t EV15 = 0;			// reserve 

volatile char RxDataChar[1];
char RxDataString[8];
//char HeaderString[] = "EBIKE-";
char HeaderString[] = "EB-";
volatile uint16_t RxDataIndex = 0;
volatile uint8_t receive_cmd = 0;
uint8_t flag_update = 0;
char buffer_MonitoringTool_Normal[300] = {0};
char buffer_MonitoringTool_AuxBattery0[300] = {0};
char buffer_MonitoringTool_AuxBattery1[300] = {0};
char buffer_MonitoringTool_MainBattery[300] = {0};

volatile uint16_t AuxBattery0_Info[27] = {0};
volatile uint16_t AuxBattery1_Info[27] = {0};
int MainBattery_Info[36] = {0};
extern FDCAN_TxHeaderTypeDef TxHeader;
													
													
uint8_t tx_data[8] = {0};
													
void query_data(){
	
	// pin ADC4_IN3 = u16_ADC4_Val[0] is raw value of VCHECK_12V 
	//Vcheck_12V = ((float)u16_ADC4_Val[0]/4095.0 * V_REF) / R93 * (R93 + R31);
	Vcheck_12V = ((float)u16_ADC4_Val[0]/4095.0 * V_REF);
	
	// pin ADC4_IN4 = u16_ADC4_Val[1] is raw value of VCHECK_5V 
	// Vcheck_5V = ((float)u16_ADC4_Val[1]/4095.0 * V_REF) / R250 * (R250 + R101);
	Vcheck_5V = ((float)u16_ADC4_Val[1]/4095.0 * V_REF);
	
	// pin ADC12_IN7 = u16_ADC2_Val[0] is raw value of AC_Chg_Input
	//	ACchg_Bat_Vin_Adc = ((float)u16_ADC2_Val[0]/4095.0 * V_REF) / R182 * (R182 + R169);
//	ACchg_Bat_Vin_Adc = ((float)u16_ADC2_Val[0]/4095.0 * V_REF);
	ACchg_Bat_Vin_Adc = ((float)u16_ADC2_Val_average[0]/4095.0 * V_REF);

	// pin ADC12_IN8 = u16_ADC2_Val[1] is raw value of Main_Chg_Batin_Adc 
	//	Main_Chg_Batin_Adc = ((float)u16_ADC2_Val[1]/4095.0 * V_REF) / R173 * (R173 + R228);
//	Main_Chg_Batin_Adc = ((float)u16_ADC2_Val[1]/4095.0 * V_REF);
	Main_Chg_Batin_Adc = ((float)u16_ADC2_Val_average[1]/4095.0 * V_REF);
	
	
	// pin ADC12_IN9 = u16_ADC2_Val[2] is raw value of Aux_Bat_Vin_Adc
	//	Aux_Bat_Vin_Adc = ((float)u16_ADC2_Val[2]/4095.0 * V_REF) / R100 * (R100 + R99);
//	Aux_Bat_Vin_Adc = ((float)u16_ADC2_Val[2]/4095.0 * V_REF);
	Aux_Bat_Vin_Adc = ((float)u16_ADC2_Val_average[2]/4095.0 * V_REF);
	Aux_Bat_Vin_Adc = 1.8;
	
	// pin ADC2_IN5 = u16_ADC2_Val_average[7] is raw value of temp_NTC1
	float V_NTC1 = ((float)u16_ADC2_Val_average[7]/4095.0 * V_REF);
	float R_NTC1 = (V_NTC1 * R168) / (V_REF - V_NTC1);
	temp_NTC1 = 1 / (1/T0 + 1/B25_100 * log(R_NTC1/R0)) - 273.15;
	// pin ADC2_IN11 = u16_ADC2_Val_average[8] is raw value of temp_NTC2
	float V_NTC2 = ((float)u16_ADC2_Val_average[8]/4095.0 * V_REF);
	float R_NTC2 = (V_NTC2 * R165) / (V_REF - V_NTC2);
	temp_NTC2 = 1 / (1/T0 + 1/B25_100 * log(R_NTC2/R0)) - 273.15;
	// pin ADC1_IN15 = u16_ADC1_Val_average[0] is raw value of temp_NTC3
	float V_NTC3 = ((float)u16_ADC1_Val_average[0]/4095.0 * V_REF);
	float R_NTC3 = (V_NTC3 * R163) / (V_REF - V_NTC3);
	temp_NTC3 = 1 / (1/T0 + 1/B25_100 * log(R_NTC3/R0)) - 273.15;
	// pin ADC1_IN12 = u16_ADC1_Val_average[1] is raw value of temp_NTC4
	float V_NTC4 = ((float)u16_ADC1_Val_average[1]/4095.0 * V_REF);
	float R_NTC4 = (V_NTC4 * R161) / (V_REF - V_NTC4);
	temp_NTC4 = 1 / (1/T0 + 1/B25_100 * log(R_NTC4/R0)) - 273.15;	
	
	// hard_code temp_NTC2 = temp_NTC3 = temp_NTC4 = temp_NTC1
	temp_NTC2 = temp_NTC1;
	temp_NTC3 = temp_NTC1;
	temp_NTC4 = temp_NTC1;

	// pin ADC2_IN17 = u16_ADC2_Val[3] is raw value of Chg_Current_Adc
//	Chg_Current_Adc = ((float)u16_ADC2_Val[3]/4095.0 * V_REF);
	Chg_Current_Adc = ((float)u16_ADC2_Val_average[3]/4095.0 * V_REF);
	// pin ADC2_IN13 = u16_ADC2_Val[4] is raw value of Chg_High_Current_Adc
//	Chg_High_Current_Adc = ((float)u16_ADC2_Val[4]/4095.0 * V_REF);
//	Chg_High_Current_Adc = ((float)u16_ADC2_Val_average[4]/4095.0 * V_REF);
	// pin ADC2_IN3 = u16_ADC2_Val[5] is raw value of Phase1_Cur_Adc
//	Phase1_Cur_Adc = ((float)u16_ADC2_Val[5]/4095.0 * V_REF);
	Phase1_Cur_Adc = ((float)u16_ADC2_Val_average[5]/4095.0 * V_REF);
	// pin ADC2_IN4 = u16_ADC2_Val[6] is raw value of Phase2_Cur_Adc
//	Phase2_Cur_Adc = ((float)u16_ADC2_Val[6]/4095.0 * V_REF);
	Phase2_Cur_Adc = ((float)u16_ADC2_Val_average[6]/4095.0 * V_REF);
}

extern CanIDHandlerMap_t CanHandlers[MAX_MESSAGE_ID];

void send_data() {

}

uint32_t err_count = 0;
void received_command() {
	if (receive_cmd == 1) {
		if (RxDataString[3] == '0') {
			
//					HAL_UART_Transmit(&huart3, (uint8_t *)"Take CMD 0", sizeof("Take CMD 0"), HAL_MAX_DELAY);
			memset(buffer_MonitoringTool_Normal, 0, sizeof(buffer_MonitoringTool_Normal));
			snprintf(buffer_MonitoringTool_Normal, 300, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%d;", \
			Aux_Bat_Vin_Adc, ACchg_Bat_Vin_Adc, Main_Chg_Batin_Adc, Phase1_Cur_Adc, Phase2_Cur_Adc, Chg_Current_Adc, \
			0.0, 0.0, 0.0, 0.0, 0.0, EV1, EV2, EV3, EV4, EV5, EV6, EV7, EV8, EV9, EV10, EV11, EV12, EV13, EV14, EV15, 1);
			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_MonitoringTool_Normal, strlen(buffer_MonitoringTool_Normal), HAL_MAX_DELAY);
			memset(RxDataString, 0, sizeof(RxDataString));
			receive_cmd = 0;
		} else if (RxDataString[3] == '1') {

//					HAL_UART_Transmit(&huart3, (uint8_t *)"Take CMD 1", sizeof("Take CMD 1"), HAL_MAX_DELAY);
			memset(buffer_MonitoringTool_AuxBattery0, 0, sizeof(buffer_MonitoringTool_AuxBattery0));
			snprintf(buffer_MonitoringTool_AuxBattery0, 300, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;", \
			AuxBattery0_Info[0], AuxBattery0_Info[1], AuxBattery0_Info[2], AuxBattery0_Info[3], AuxBattery0_Info[4], AuxBattery0_Info[5], AuxBattery0_Info[6], AuxBattery0_Info[7], 
			AuxBattery0_Info[8], AuxBattery0_Info[9], AuxBattery0_Info[10], AuxBattery0_Info[11], AuxBattery0_Info[12], AuxBattery0_Info[13], AuxBattery0_Info[14], AuxBattery0_Info[15], 
			AuxBattery0_Info[16], AuxBattery0_Info[17], AuxBattery0_Info[18], AuxBattery0_Info[19], AuxBattery0_Info[20], AuxBattery0_Info[21], AuxBattery0_Info[22], AuxBattery0_Info[23], 
			AuxBattery0_Info[24], AuxBattery0_Info[25], AuxBattery0_Info[26]);
			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_MonitoringTool_AuxBattery0, strlen(buffer_MonitoringTool_AuxBattery0), HAL_MAX_DELAY);
			memset(RxDataString, 0, sizeof(RxDataString));
			receive_cmd = 0;
		} else if (RxDataString[3] == '2') {
//					HAL_UART_Transmit(&huart3, (uint8_t *)"Take CMD 2", sizeof("Take CMD 2"), HAL_MAX_DELAY);
			memset(buffer_MonitoringTool_AuxBattery1, 0, sizeof(buffer_MonitoringTool_AuxBattery1));
			snprintf(buffer_MonitoringTool_AuxBattery1, 300, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;", \
			AuxBattery1_Info[0], AuxBattery1_Info[1], AuxBattery1_Info[2], AuxBattery1_Info[3], AuxBattery1_Info[4], AuxBattery1_Info[5], AuxBattery1_Info[6], AuxBattery1_Info[7], 
			AuxBattery1_Info[8], AuxBattery1_Info[9], AuxBattery1_Info[10], AuxBattery1_Info[11], AuxBattery1_Info[12], AuxBattery1_Info[13], AuxBattery1_Info[14], AuxBattery1_Info[15], 
			AuxBattery1_Info[16], AuxBattery1_Info[17], AuxBattery1_Info[18], AuxBattery1_Info[19], AuxBattery1_Info[20], AuxBattery1_Info[21], AuxBattery1_Info[22], AuxBattery1_Info[23], 
			AuxBattery1_Info[24], AuxBattery1_Info[25], AuxBattery1_Info[26]);
			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_MonitoringTool_AuxBattery1, strlen(buffer_MonitoringTool_AuxBattery1), HAL_MAX_DELAY);
			memset(RxDataString, 0, sizeof(RxDataString));
			receive_cmd = 0;
		} else if (RxDataString[3] == '3') {
//					HAL_UART_Transmit(&huart3, (uint8_t *)"Take CMD 3", sizeof("Take CMD 3"), HAL_MAX_DELAY);
			snprintf(buffer_MonitoringTool_MainBattery, 300, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d;", \
			MainBattery_Info[0], MainBattery_Info[1], MainBattery_Info[2], MainBattery_Info[3], MainBattery_Info[4], MainBattery_Info[5], MainBattery_Info[6], MainBattery_Info[7], 
			MainBattery_Info[8], MainBattery_Info[9], MainBattery_Info[10], MainBattery_Info[11], MainBattery_Info[12], MainBattery_Info[13], MainBattery_Info[14], MainBattery_Info[15], 
			MainBattery_Info[16], MainBattery_Info[17], MainBattery_Info[18], MainBattery_Info[19], MainBattery_Info[20], MainBattery_Info[21], MainBattery_Info[22], MainBattery_Info[23], 
			MainBattery_Info[24], MainBattery_Info[25], MainBattery_Info[26], MainBattery_Info[27], MainBattery_Info[28], MainBattery_Info[29], MainBattery_Info[30], MainBattery_Info[31],
			MainBattery_Info[32], MainBattery_Info[33], MainBattery_Info[34], MainBattery_Info[35]);
			HAL_UART_Transmit(&huart3, (uint8_t *)buffer_MonitoringTool_MainBattery, strlen(buffer_MonitoringTool_MainBattery), HAL_MAX_DELAY);
			memset(RxDataString, 0, sizeof(RxDataString));
			receive_cmd = 0;
		} else if (RxDataString[3] == '4') {
			// Receive command update EBIKE-4, want to reset to back bootloader 
//			HAL_UART_Transmit(&huart3, (uint8_t *)"Take CMD 4", sizeof("Take CMD 4"), HAL_MAX_DELAY);
			if (flag_update == 0) {
				flag_update = 1;
				next_stage = stage1;						// Set Charger off all output, to prepare update firmware
			} else {
				memset(RxDataString, 0, sizeof(RxDataString));
				receive_cmd = 0;
				HAL_NVIC_SystemReset();
			}
		}	else {
#ifdef DEBUG
			HAL_UART_Transmit(&huart3, (uint8_t *)"No support", sizeof("No support"), HAL_MAX_DELAY);
#endif
			memset(RxDataString, 0, sizeof(RxDataString));
			receive_cmd = 0;
		}
		
	
	}	
}