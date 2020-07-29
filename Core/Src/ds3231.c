/*
 * ds3231.c
 *
 *  Created on: Jul 29, 2020
 *      Author: lauer
 */

#include "ds3231.h"

void alarmOneInit(I2C_HandleTypeDef *hi2c) {
	uint8_t alarmOneData[4];
	alarmOneData[0] = (1 << A1M1) | 0b00100000;
	alarmOneData[1] = 0b00000000;
	alarmOneData[2] = 0b00000000;
	alarmOneData[3] = 0b00000000;

	HAL_I2C_Mem_Write(hi2c, DS3231_ADDR, 0x07, 1, alarmOneData, 4, 1000);

	uint8_t controlReg = (1 << INTCN) | (1 << A1IE);
//	uint8_t controlReg = 0b00000000;
	HAL_I2C_Mem_Write(hi2c, DS3231_ADDR, 0x0E, 1, &controlReg, 1, 1000);

	uint8_t statusReg = 0b00000000;
	HAL_I2C_Mem_Write(hi2c, DS3231_ADDR, 0x0F, 1, &statusReg, 1, 1000);
}

void setAlarmOne(I2C_HandleTypeDef *hi2c, uint8_t alarmHour, uint8_t alarmMinute, uint8_t alarmAmPm) {
	uint8_t alarmOneData[4];
	alarmOneData[0] = 0x00;
	alarmOneData[1] = 0x7F & alarmMinute;
	if(alarmAmPm == 1) {
		alarmOneData[2] = (1 << 6) | (1 << 5) | alarmHour;
	} else if (alarmAmPm == 0) {
		alarmOneData[2] = (1 << 6) | alarmHour;
	}
	alarmOneData[3] = (1 << A1M4) | 0x00;

	HAL_I2C_Mem_Write(hi2c, DS3231_ADDR, 0x07, 1, alarmOneData, 4, 1000);

}

void setTime(I2C_HandleTypeDef *hi2c) {
	uint8_t setTime[7];

	setTime[0] = decToBcd(54);
	setTime[1] = decToBcd(0);
	setTime[2] = 0b01000000 | decToBcd(7); // 12 hour mode
	setTime[3] = decToBcd(2);
	setTime[4] = decToBcd(27);
	setTime[5] = decToBcd(7);
	setTime[6] = decToBcd(20);
	HAL_I2C_Mem_Write(hi2c, DS3231_ADDR, 0x00, 1, setTime, 7, 1000);
}

void getTime(I2C_HandleTypeDef *hi2c, Time *time) {
	uint8_t getTime[7];

	HAL_I2C_Mem_Read(hi2c, DS3231_ADDR, 0x00, 1, getTime, 7, 100);

	time->seconds = bcdToDec(getTime[0]);
	time->minutes = bcdToDec(getTime[1]);
	time->hours = bcdToDec(0x1F & getTime[2]);
	time->dow = bcdToDec(getTime[3]);
	time->dom = bcdToDec(getTime[4]);
	time->month = bcdToDec(getTime[5]);
	time->year = bcdToDec(getTime[6]);
	if((getTime[2] & 0x20) == 0) {
		time->amPm = 0;
	} else if((getTime[2] & 0x20) == 0x20) {
		time->amPm = 1;
	}
}

uint8_t decToBcd(int val) {
	return (uint8_t)((val/10*16) + (val%10));
}

int bcdToDec(uint8_t val) {
	return (int)((val/16*10) + (val%16));
}
