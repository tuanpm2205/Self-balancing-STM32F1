#ifndef __DHT_11
#define __DHT_11
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define  DHT11_PORT  GPIOB
#define  DHT11_PIN   GPIO_PIN_1

void Delay_Microsecond(uint16_t us);
void PIN_DHT_tooutput(void);
void PIN_DHT_toinput(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
void DHT11_Read(void);
uint8_t DHT11_Check(void);
void DHT11_Start(void);

typedef struct{
	uint8_t IntHum;
	uint8_t FloatHum;
	uint8_t IntTemp;
	uint8_t FloatTemp;
	uint8_t Checksum;
} DHT_Data;

extern DHT_Data DHT;

#endif