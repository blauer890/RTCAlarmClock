/*
 * ds3231.h
 *
 *  Created on: Jul 29, 2020
 *      Author: lauer
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define A1M1 7
#define A1M2 7
#define A1M3 7
#define A1M4 7
#define INTCN 2 // Interrupt control
#define A1IE 0 // Alarm 1 interrupt enable
#define A2IE 1 // Alarm 2 interrupt enable
#define A1F 0
#define DS3231_ADDR (0x68 << 1)

typedef struct Times {
	int seconds;
	int minutes;
	int hours;
	int dow;
	int dom;
	int month;
	int year;
	int amPm;
} Time;

void alarmOneInit(I2C_HandleTypeDef *hi2c);
void setAlarmOne(I2C_HandleTypeDef *hi2c, uint8_t alarmHour, uint8_t alarmMinute, uint8_t alarmAmPm);
void setTime(I2C_HandleTypeDef *hi2c);
void getTime(I2C_HandleTypeDef *hi2c, Time *time);
uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);

#endif /* INC_DS3231_H_ */
