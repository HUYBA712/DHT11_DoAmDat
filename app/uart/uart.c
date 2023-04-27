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
		if(huart_data.buff[i] == '[')
		{
			for(j=i+1;j<huart_data.index;j++)
			{
				if(huart_data.buff[j] == ']')
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
		len = j - i + 1;
		memcpy((void*)data_out,(const void*)(huart_data.buff + i), len);
		huart_data.index = 0;
	}
	return len;
}

extern bool isOnAuto_relay;

void uart_process()
{
	uint8_t data_rx_1[10] = {0};
	uint16_t length_1 = 0;
	length_1 = uart_get_dataFromBuffer((uint8_t*)data_rx_1);
	if(length_1 == 0) return;
	if(data_rx_1[length_1-2] == uart_calculateChecksum((data_rx_1+1),length_1 - 3))
	{
		if(isOnAuto_relay)
		{
			if((data_rx_1[1] == 0x01) && (data_rx_1[2] == 0x03))
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);
			}
			else if((data_rx_1[1] == 0x01) && (data_rx_1[2] == 0x04))
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
			}
			if((data_rx_1[1] == 0x02) && (data_rx_1[2] == 0x03))
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1);
			}
			else if((data_rx_1[1] == 0x02) && (data_rx_1[2] == 0x04))
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,0);
			}
		}
		
		if((data_rx_1[1] == 0x03) && (data_rx_1[2] == 0x03))
		{
			isOnAuto_relay = true;
		}
		else if((data_rx_1[1] == 0x03) && (data_rx_1[2] == 0x04))
		{
			isOnAuto_relay = false;
		}
		
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
	



