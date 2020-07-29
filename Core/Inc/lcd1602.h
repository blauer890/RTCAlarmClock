/*
 * lcd1602.h
 *
 *  Created on: Jul 29, 2020
 *      Author: lauer
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define RS 0
#define RW 1
#define EN 2
#define LED 3 // Back light anode (5V)
#define DB4 4
#define DB5 5
#define DB6 6
#define DB7 7
#define CR 0
#define DR 1

#define LCD6502_ADDR (0x27 << 1)

void writeNibble(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t nibble);
void writeByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t byte);
uint8_t readNibble(I2C_HandleTypeDef *hi2c, uint8_t reg);
uint8_t readByte(I2C_HandleTypeDef *hi2c, uint8_t reg);
void initLCD(I2C_HandleTypeDef *hi2c);
void lcd_putc(I2C_HandleTypeDef *hi2c, uint8_t c);
void lcd_puts(I2C_HandleTypeDef *hi2c, uint8_t *s, uint16_t size);
void lcd_puti(I2C_HandleTypeDef *hi2c, int num);

#endif /* INC_LCD1602_H_ */
