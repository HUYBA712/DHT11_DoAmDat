#include "doam_dat.h"
#include "stm32f1xx_hal.h"





adc_type_t state_adc;
static ADC_HandleTypeDef hAdc_1;
volatile uint8_t g_adc_flag = 0;

void doam_dat_init(ADC_HandleTypeDef hadc)
{
	hAdc_1 = hadc;
	HAL_ADCEx_Calibration_Start(&hAdc_1);
}



void doam_dat_start(ADC_HandleTypeDef hadc)
{
	hAdc_1 = hadc;
	HAL_ADC_Start_IT(&hAdc_1);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint8_t icount = 0;
	state_adc.sum[icount] += HAL_ADC_GetValue(hadc);
	state_adc.index[icount]++;
	if(state_adc.index[icount] == MAX_SAMPLE_ADC)
	{
		state_adc.value[icount] = state_adc.sum[icount] / MAX_SAMPLE_ADC;
		state_adc.sum[icount] = 0;
		g_adc_flag = 1;
		state_adc.index[icount] = 0;
	}
	icount++;
	if(icount>=4)
	{
		icount=0;
	}
	HAL_ADC_Stop_IT(hadc);
}


uint32_t doam_dat_getValue_sensor1()
{
	return state_adc.value[0];
}


uint32_t doam_dat_getValue_sensor2()
{
	return state_adc.value[1];
}


uint32_t doam_dat_getvalue_anhsang1()
{
	return state_adc.value[2];
}


uint32_t doam_dat_getvalue_anhsang2()
{
	return state_adc.value[3];
}





