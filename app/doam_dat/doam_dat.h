#ifndef DOAM_DAT_H_
#define DOAM_DAT_H_

#include "stm32f1xx_hal.h"
#include "main.h"



#define MAX_SAMPLE_ADC 5

typedef struct{
	uint32_t sum[4];//tinh tong ADC
	uint32_t value[4];
	uint32_t timeADC;//time luu ADC
	uint8_t index[4];
}adc_type_t;




void doam_dat_init(ADC_HandleTypeDef hadc);
void doam_dat_start(ADC_HandleTypeDef hadc);
uint32_t doam_dat_getValue_sensor1();
uint32_t doam_dat_getValue_sensor2();

uint32_t doam_dat_getvalue_anhsang1();
uint32_t doam_dat_getvalue_anhsang2();



#endif
