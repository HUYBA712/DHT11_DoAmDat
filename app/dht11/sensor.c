#include "sensor.h"
#include "main.h"
#include "stm32f1xx_hal.h"




static void sensor_DHT11_1_Mode_OUT_PP(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = DHT11_1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
  HAL_GPIO_Init(DHT11_1_PORT, &GPIO_InitStruct);
}
 

static void sensor_DHT11_1_Mode_IN_NP(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = DHT11_1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
 
  HAL_GPIO_Init(DHT11_1_PORT, &GPIO_InitStruct);
}


static void sensor_DHT11_2_Mode_OUT_PP(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = DHT11_2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
  HAL_GPIO_Init(DHT11_2_PORT, &GPIO_InitStruct);
}
 

static void sensor_DHT11_2_Mode_IN_NP(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = DHT11_2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
 
  HAL_GPIO_Init(DHT11_2_PORT, &GPIO_InitStruct);
}


uint8_t DHT11_1_ReadByte(void)
{
	uint8_t i, temp = 0;
 
	for (i = 0; i < 8; i++)
	{
		while (DHT11_1_IN == 0); // doi muc thap ket thuc
		
		delay_us(40); // tre 40us
		
		if (DHT11_1_IN == 1)
		{
			while (DHT11_1_IN == 1); // doi muc cao ket thuc
			
			temp |= (uint8_t)(0X01 << (7 - i)); // gui MSB cao truoc
		}
		else
		{
			temp &= (uint8_t)~(0X01 << (7 - i));
		}
	}
	return temp;
}



uint8_t DHT11_2_ReadByte(void)
{
	uint8_t i, temp = 0;
 
	for (i = 0; i < 8; i++)
	{
		while (DHT11_2_IN == 0); // doi muc thap ket thuc
		
		delay_us(40); // tre 40us
		
		if (DHT11_2_IN == 1)
		{
			while (DHT11_2_IN == 1); // doi muc cao ket thuc
			
			temp |= (uint8_t)(0X01 << (7 - i)); // gui MSB cao truoc
		}
		else
		{
			temp &= (uint8_t)~(0X01 << (7 - i));
		}
	}
	return temp;
}








uint8_t DHT11_1_ReadData(DHT11_Data_TypeDef *DHT11_Data)
{
	sensor_DHT11_1_Mode_OUT_PP(); // set OUT PUT, set xuong muc 0
	DHT11_1_OUT_0;	
	delay_us(18000);  // tre 18ms
	
	DHT11_1_OUT_1; // set len muc 1, tre 30us 
	delay_us(30);	
 
	sensor_DHT11_1_Mode_IN_NP(); // chuyen sang IN PUT, de nhan data
	
	if (DHT11_1_IN == 0)
	{
		while (DHT11_1_IN == 0);
		
		while (DHT11_1_IN == 1);		
		
		// bat dau nhan data
		DHT11_Data->humi_int  = DHT11_1_ReadByte();
		DHT11_Data->humi_deci = DHT11_1_ReadByte();
		DHT11_Data->temp_int  = DHT11_1_ReadByte();
		DHT11_Data->temp_deci = DHT11_1_ReadByte();
		DHT11_Data->check_sum = DHT11_1_ReadByte();
		
		sensor_DHT11_1_Mode_OUT_PP();	// khi doc xong, set OUTPUT r set muc 1
		DHT11_1_OUT_1;	
		
		// xac nhan data
		if (DHT11_Data->check_sum == DHT11_Data->humi_int + DHT11_Data->humi_deci + DHT11_Data->temp_int + DHT11_Data->temp_deci)	
		{
			return 1;
		}		
		else
		{
			return 0;
		}
	}
	else		
	{
		return 0;
	}
}









uint8_t DHT11_2_ReadData(DHT11_Data_TypeDef *DHT11_Data)
{
	sensor_DHT11_2_Mode_OUT_PP(); // set OUT PUT, set xuong muc 0
	DHT11_2_OUT_0;	
	delay_us(18000); // tre 18ms
	
	DHT11_2_OUT_1; // set len muc 1, tre 30us 
	delay_us(30);	
 
	sensor_DHT11_2_Mode_IN_NP(); // chuyen sang IN PUT, de nhan data
	
	if (DHT11_2_IN == 0)
	{
		while (DHT11_2_IN == 0);
		
		while (DHT11_2_IN == 1);		
		
		// bat dau nhan data
		DHT11_Data->humi_int  = DHT11_2_ReadByte();
		DHT11_Data->humi_deci = DHT11_2_ReadByte();
		DHT11_Data->temp_int  = DHT11_2_ReadByte();
		DHT11_Data->temp_deci = DHT11_2_ReadByte();
		DHT11_Data->check_sum = DHT11_2_ReadByte();
		
		sensor_DHT11_2_Mode_OUT_PP();	// khi doc xong, set OUTPUT r set muc 1
		DHT11_2_OUT_1;	
		
		// xac nhan data
		if (DHT11_Data->check_sum == DHT11_Data->humi_int + DHT11_Data->humi_deci + DHT11_Data->temp_int + DHT11_Data->temp_deci)	
		{
			return 1;
		}		
		else
		{
			return 0;
		}
	}
	else		
	{
		return 0;
	}
}















