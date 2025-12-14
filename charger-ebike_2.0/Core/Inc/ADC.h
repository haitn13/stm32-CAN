#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "common.h"

	
void ADC_CONFIG_COMMON(ADC_HandleTypeDef *hadc, uint32_t Channel, uint32_t SampleTime);
void ADC1_Select_CH12 (void);
void ADC1_Select_CH15 (void);
void ADC2_Select_CH3 (void);
void ADC2_Select_CH4 (void);
void ADC2_Select_CH5 (void);
void ADC2_Select_CH7 (void);
void ADC2_Select_CH8 (void);
void ADC2_Select_CH9 (void);
void ADC2_Select_CH11 (void);
void ADC2_Select_CH13 (void);
void ADC2_Select_CH17 (void);
void ADC4_Select_CH3 (void);
void ADC4_Select_CH4 (void);


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */