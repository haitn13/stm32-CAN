#include "ADC.h"


// ADC code
void ADC_CONFIG_COMMON(ADC_HandleTypeDef *hadc, uint32_t Channel, uint32_t SampleTime)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = Channel;
  sConfig.SamplingTime = SampleTime;
	// Mode single diff, offset none, offset = 0 for all ADC
	sConfig.Rank = ADC_REGULAR_RANK_1; // MUST SET TO 1 FOR ALL
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC1_Select_CH12 (void)
{
	ADC_CONFIG_COMMON(&hadc1, ADC_CHANNEL_12, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC1_Select_CH15 (void)
{
	ADC_CONFIG_COMMON(&hadc1, ADC_CHANNEL_15, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH3 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_3, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH4 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_4, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH5 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH7 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_7, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH8 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_8, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH9 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_9, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH11 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_11, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH13 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_13, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC2_Select_CH17 (void)
{
	ADC_CONFIG_COMMON(&hadc2, ADC_CHANNEL_17, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC4_Select_CH3 (void)
{
	ADC_CONFIG_COMMON(&hadc4, ADC_CHANNEL_3, ADC_SAMPLETIME_24CYCLES_5);
}

void ADC4_Select_CH4 (void)
{
	ADC_CONFIG_COMMON(&hadc4, ADC_CHANNEL_4, ADC_SAMPLETIME_24CYCLES_5);
}