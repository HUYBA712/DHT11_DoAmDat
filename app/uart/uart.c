#include "uart.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdbool.h"


Uart_Typedef huart_data;
static uint8_t data_rx;

static UART_HandleTypeDef* hUart_1;




void uart_init(UART_HandleTypeDef *huart)
{
	hUart_1 = huart;
	HAL_UART_Receive_IT(hUart_1,&data_rx,1);
}




void uart_transmitData(char *data,uint16_t lenght)
{
	char buf_to_esp[100];
	uint8_t cks = 0;
	cks = uart_calculateChecksum((uint8_t*)data, lenght);
	strcpy((buf_to_esp + 1), data);
	buf_to_esp[0] = '[';
	buf_to_esp[lenght + 1] = cks;
	buf_to_esp[lenght + 2] = ']';
	HAL_UART_Transmit(hUart_1,(uint8_t*)buf_to_esp, strlen(buf_to_esp),100);
}


uint8_t uart_calculateChecksum(uint8_t *buf,uint16_t len)
{
	uint8_t chk = 0;
	uint16_t i = 0;
	for(i=0;i<len;i++)
	{
			chk ^= buf[i];
	}
	return chk;
}



uint16_t uart_get_dataFromBuffer(uint8_t* data_out)
{
	if(huart_data.index < 4) return 0;
	bool haveHdrAndTmn = false;
	uint8_t i ,j;
	for(i=0;i<huart_data.index-1;i++)
	{
		if(huart_data.buff[i] == 0x01)
		{
			for(j=i+1;j<huart_data.index;j++)
			{
				if(huart_data.buff[j] == 0x02)
				{
					haveHdrAndTmn = true;
					break;
				}
			}
			break;
		}
	}
	uint16_t len = 0;
	if(haveHdrAndTmn)
	{
		len = j - i;
		memcpy((void*)data_out,(const void*)(huart_data.buff + i), len);
		huart_data.index = 0;
	}
	return len;
}


void uart_process()
{
	uint8_t data_rx[100] = {0};
	uint16_t length = 0;
	length = uart_get_dataFromBuffer((uint8_t*)data_rx);
	if(length == 0) return;
	if(data_rx[length-2] == uart_calculateChecksum((data_rx+1),length - 3))
	{
		
	}
	else
	{
		return;
	}
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == hUart_1->Instance)
	{
		if(huart_data.index < MAX_BUFFER_SIZE)
		{
			huart_data.buff[huart_data.index++] = data_rx;
		}
		else
		{
			huart_data.index = 0;
		}
		
		HAL_UART_Receive_IT(hUart_1,&data_rx,1);
	}
}
	



