#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f1xx_hal.h"
#include "main.h"


#define DHT11_1_PORT		   GPIOA
#define DHT11_1_PIN		      GPIO_PIN_0
 
#define DHT11_1_OUT_1				HAL_GPIO_WritePin(DHT11_1_PORT, DHT11_1_PIN, GPIO_PIN_SET)
#define DHT11_1_OUT_0				HAL_GPIO_WritePin(DHT11_1_PORT, DHT11_1_PIN, GPIO_PIN_RESET)
 
#define DHT11_1_IN					        HAL_GPIO_ReadPin(DHT11_1_PORT, DHT11_1_PIN)
 
 
#define DHT11_2_PORT		   GPIOA
#define DHT11_2_PIN		      GPIO_PIN_1
 
#define DHT11_2_OUT_1				HAL_GPIO_WritePin(DHT11_2_PORT, DHT11_2_PIN, GPIO_PIN_SET)
#define DHT11_2_OUT_0				HAL_GPIO_WritePin(DHT11_2_PORT, DHT11_2_PIN, GPIO_PIN_RESET)
 
#define DHT11_2_IN					        HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN)
 

typedef struct
{
	uint8_t humi_int;			
	uint8_t humi_deci;	 		
	uint8_t temp_int;	 			
	uint8_t temp_deci;	 		
	uint8_t check_sum;	 		
		                 
} DHT11_Data_TypeDef;



uint8_t DHT11_1_ReadData(DHT11_Data_TypeDef *DHT11_Data);
uint8_t DHT11_2_ReadData(DHT11_Data_TypeDef *DHT11_Data);



#endif
