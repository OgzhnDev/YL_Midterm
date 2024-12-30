/*
 * myDS1302Lib.c
 *
 *  Created on: Dec 10, 2024
 *      Author: oguzh
 */

// 3.parti kütüphane

#include "myDS1302Lib.h"


//void // delayUS_DWT(uint32_t us)
//{
//	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
//	volatile uint32_t start = DWT->CYCCNT;
//	do  {
//	} while(DWT->CYCCNT - start < cycles);
//}

void writeSDA(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = DS1302_SDA;
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);

}

void readSDA(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = DS1302_SDA;
	GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);
}

void DS1302_SendCmd(uint8_t cmd)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
//		DS1302_SDA = (bit)(addr & 1);
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (cmd & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
//		DS1302_SCK = 1;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_SET);
		// delayUS_DWT(1);
//		DS1302_SCK = 0;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);
		// delayUS_DWT(1);
		cmd >>= 1;
	}
}

void DS1302_WriteByte(uint8_t addr, uint8_t d)
{
	uint8_t i;

//	DS1302_RST = 1;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST,  GPIO_PIN_SET);

	//addr = addr & 0xFE;
	DS1302_SendCmd(addr);	// Sends address

	for (i = 0; i < 8; i ++)
	{
//		DS1302_SDA = (bit)(d & 1);
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (d & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
//		DS1302_SCK = 1;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_SET);
		// delayUS_DWT(1);
//		DS1302_SCK = 0;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);
		// delayUS_DWT(1);
		d >>= 1;
	}

//	DS1302_RST = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST,  GPIO_PIN_RESET);
	//	DS1302_SDA = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA,  GPIO_PIN_RESET);
}

uint8_t DS1302_ReadByte(uint8_t addr)
{
	uint8_t i;
	uint8_t temp = 0;

//	DS1302_RST = 1;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST,  GPIO_PIN_SET);
	addr = addr | 0x01; // Generate Read Address

	DS1302_SendCmd(addr);	// Sends address

	readSDA();
	for (i = 0; i < 8; i ++)
	{
		temp >>= 1;
//		if(DS1302_SDA)
		if(HAL_GPIO_ReadPin(DS1302_GPIO, DS1302_SDA))
			temp |= 0x80;
//		DS1302_SCK = 1;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_SET);
		// delayUS_DWT(1);
//		DS1302_SCK = 0;
		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);
		// delayUS_DWT(1);
	}
	writeSDA();

//	DS1302_RST = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST,  GPIO_PIN_RESET);
//	DS1302_SDA = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA,  GPIO_PIN_RESET);
	return temp;
}

void DS1302_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = DS1302_SCLK | DS1302_SDA | DS1302_RST;
    GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStructure);

	DS1302_WriteByte(DS1302_CHARGER,0x00);			// Disable Trickle Charger

//	DS1302_RST = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST,  GPIO_PIN_RESET);
//	DS1302_SCK = 0;
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);

	// DWT->CTRL |= 1 ; // enable the counter for microsecond delay, see "void // delayUS_DWT(uint32_t us)"
}

void DS1302_ReadTime(uint8_t *buf)
{
   	uint8_t tmp;

	tmp = DS1302_ReadByte(DS1302_YEAR);
	buf[1] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte(DS1302_MONTH);
	buf[2] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte(DS1302_DATE);
	buf[3] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte(DS1302_HOUR);
	buf[4] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte(DS1302_MIN);
	buf[5] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte((DS1302_SEC))&0x7F;
	buf[6] = BCD2HEX(tmp);
	tmp = DS1302_ReadByte(DS1302_DAY);
	buf[7] = BCD2HEX(tmp);
}

void DS1302_WriteTime(uint8_t *buf)
{
	DS1302_WriteByte(DS1302_CONTROL,0x00);			// Disable write protection
	// delayUS_DWT(1);
	DS1302_WriteByte(DS1302_SEC,0x80);
	DS1302_WriteByte(DS1302_YEAR,HEX2BCD(buf[1]));
	DS1302_WriteByte(DS1302_MONTH,HEX2BCD(buf[2]));
	DS1302_WriteByte(DS1302_DATE,HEX2BCD(buf[3]));
	DS1302_WriteByte(DS1302_HOUR,HEX2BCD(buf[4]));
	DS1302_WriteByte(DS1302_MIN,HEX2BCD(buf[5]));
	DS1302_WriteByte(DS1302_SEC,HEX2BCD(buf[6]));
	DS1302_WriteByte(DS1302_DAY,HEX2BCD(buf[7]));
	DS1302_WriteByte(DS1302_CONTROL,0x80);			// Enable write protection
	// delayUS_DWT(1);
}


