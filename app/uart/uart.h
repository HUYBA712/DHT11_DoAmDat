#ifndef UART_H_
#define UART_H_

#include "stm32f1xx_hal.h"
#include "main.h"



#define MAX_BUFFER_SIZE 100
typedef struct
{
	uint8_t flag;
	uint8_t buff[100];
	uint8_t index;
}Uart_Typedef;

void uart_init(UART_HandleTypeDef huart);

void uart_transmitData(UART_HandleTypeDef huart,char *data);
uint8_t uart_calculateChecksum(uint8_t *buf,uint16_t len);

void uart_process();


#endif
