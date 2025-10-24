#include "DHT11.h"

extern TIM_HandleTypeDef htim4;
DHT_Data DHT;

void Delay_Microsecond(uint16_t us){
	HAL_TIM_Base_Start(&htim4);
	htim4.Instance->CNT = 0;
	while(htim4.Instance->CNT < us*2){}
	HAL_TIM_Base_Stop(&htim4);
}

void PIN_DHT_tooutput(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}
void PIN_DHT_toinput(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}
uint8_t DHT11_ReadBit(void){
	uint8_t Timeout = 100;
	uint8_t Time = 0;
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==0  && Time < Timeout){
		Time++;
		Delay_Microsecond(1);
	}
	Delay_Microsecond(40);
	Time = 0;
	if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==1){
		Time = 0;
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==1  && Time < Timeout){
			Time++;
			Delay_Microsecond(1);
		}
		return 1;
	}else return 0;
}
uint8_t DHT11_ReadByte(void){
	uint8_t data=0;
	for(int8_t i = 7; i > -1; i--){
		data |= DHT11_ReadBit()<<i;
	}
	return data;
}
void DHT11_Read(void){
	// DHT11 Start
	uint16_t Timeout = 50000;
	uint16_t Time = 0;
	PIN_DHT_tooutput();
	Delay_Microsecond(10);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	HAL_Delay(20);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	Delay_Microsecond(10);
	PIN_DHT_toinput();
	Delay_Microsecond(10);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==1 && Time < Timeout){
		Time++;
		Delay_Microsecond(1);
	}
	Time = 0;
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==0 && Time < Timeout){
		Time++;
		Delay_Microsecond(1);
	}
	Time = 0;
	Delay_Microsecond(10);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==1 && Time < Timeout){
		Time++;
		Delay_Microsecond(1);
	}
	// Read Data
	DHT.IntHum = DHT11_ReadByte();
	DHT.FloatHum = DHT11_ReadByte();
	DHT.IntTemp = DHT11_ReadByte();
	DHT.FloatTemp = DHT11_ReadByte();
	DHT.Checksum = DHT11_ReadByte();
}