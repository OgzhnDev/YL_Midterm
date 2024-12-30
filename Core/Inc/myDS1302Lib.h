/*
 * myDS1302Lib.h
 *
 *  Created on: Dec 10, 2024
 *      Author: oguzh
 */

#ifndef INC_MYDS1302LIB_H_
#define INC_MYDS1302LIB_H_

#include "stm32f4xx_hal.h"

// Entegre register adresleri
#define DS1302_SEC			0x80
#define DS1302_MIN			0x82
#define DS1302_HOUR			0x84
#define DS1302_DATE			0x86
#define DS1302_MONTH		0x88
#define DS1302_DAY			0x8a
#define DS1302_YEAR			0x8c
#define DS1302_CONTROL		0x8e
#define DS1302_CHARGER		0x90
#define DS1302_CLKBURST		0xbe
#define DS1302_RAMBURST 	0xfe


// Haberlesme icin GPIO Pinleri, bu entegre I2C desteklemez
#define DS1302_GPIO	GPIOB
#define DS1302_SCLK	GPIO_PIN_0
#define DS1302_SDA	GPIO_PIN_1
#define DS1302_RST	GPIO_PIN_2


// Hex'ten bcd'ye gecis, entegre degerleri BCD tutuyor
#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)

// Entegre icin init fonksiyonu
void DS1302_Init(void);

// Zamanı byte seklinde okuyup buffera yazan fonksiyon
void DS1302_ReadTime(uint8_t *buf);

// Buffera byte seklinde zaman yazan fonksiyon
void DS1302_WriteTime(uint8_t *buf);

// Byte yazma islemi icin fonksiyon
void DS1302_WriteByte(uint8_t,uint8_t);

// Byte okuma islemi icin fonksiyon
uint8_t DS1302_ReadByte(uint8_t);

// Komut gönderebilmek icin fonksiyon
void DS1302_SendCmd(uint8_t);

#endif /* INC_MYDS1302LIB_H_ */
