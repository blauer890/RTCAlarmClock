/*
 * lcd1602.c
 *
 *  Created on: Jul 29, 2020
 *      Author: lauer
 */

#include "lcd1602.h"

void writeNibble(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t nibble) {
	uint8_t setup = 0x00;
	uint8_t data = 0x00;
	uint8_t closing = 0x00;

	if(reg == 1) {
		setup = 0b00001101;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &setup, 1, 100);
		HAL_Delay(10);
		data = (nibble << 4) | 0b00001101;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &data, 1, 100);
		HAL_Delay(10);
		closing = (nibble << 4) | 0b00001001;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &closing, 1, 100);
	} else if (reg == 0) {
		setup = 0b00001100;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &setup, 1, 100);
		HAL_Delay(1);
		data = (nibble << 4) | 0b00001100;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &data, 1, 100);
		HAL_Delay(1);
		closing = (nibble << 4) | 0b00001000;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &closing, 1, 100);
	}
}

void writeByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t byte) {
	uint8_t highNibble = 0x0F & (byte >> 4);
	uint8_t lowNibble = 0x0F & byte;
	writeNibble(hi2c, reg, highNibble);
	HAL_Delay(5);
	writeNibble(hi2c, reg, lowNibble);
}

void initLCD(I2C_HandleTypeDef *hi2c) {
//	HAL_Delay(20);
//	writeNibble(hi2c, CR, 0b00000011);
//	HAL_Delay(4);
//	writeNibble(hi2c, CR, 0b00000011);
//	HAL_Delay(1);
//	writeNibble(hi2c, CR, 0b00000011);
//	HAL_Delay(1);
	writeNibble(hi2c, CR, 0b00000010);
//	HAL_Delay(10);
	writeByte(hi2c, CR, 0b00101000);
//	HAL_Delay(10);
	writeByte(hi2c, CR, 0b00001100);
//	HAL_Delay(10);
	writeByte(hi2c, CR, 0b00000001);
//	HAL_Delay(10);	//       DL/N/
}

uint8_t readNibble(I2C_HandleTypeDef *hi2c, uint8_t reg) {
//	HAL_I2C_Master_Receive(hi2c, LCD6502_ADDR, &data, strlen(data), 100);
	uint8_t setup = 0x00;
	uint8_t data = 0x00;
	uint8_t closing = 0x00;

	if(reg == 1) {
		setup = 0b00001111;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &setup, 1, 100);
		HAL_Delay(1);
		HAL_I2C_Master_Receive(hi2c, LCD6502_ADDR, &data, 1, 100);
		HAL_Delay(1);
		closing = 0b00001011;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &closing, 1, 100);
	} else if (reg == 0) {
		setup = 0b00001110;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &setup, 1, 100);
		HAL_Delay(1);
		HAL_I2C_Master_Receive(hi2c, LCD6502_ADDR, &data, 1, 100);
		HAL_Delay(1);
		closing = 0b00001010;
		HAL_I2C_Master_Transmit(hi2c, LCD6502_ADDR, &closing, 1, 100);
	}
	return data;
}

uint8_t readByte(I2C_HandleTypeDef *hi2c, uint8_t reg) {
	uint8_t highNibble = 0x00;
	uint8_t lowNibble = 0x00;
	uint8_t byte = 0x00;
	highNibble = readNibble(hi2c, reg);
	HAL_Delay(1);
	lowNibble = readNibble(hi2c, reg);
	byte = (highNibble << 4) | lowNibble;
	return byte;
}

void lcd_putc(I2C_HandleTypeDef *hi2c, uint8_t c) {
	writeByte(hi2c, DR, c);
}

void lcd_puts(I2C_HandleTypeDef *hi2c, uint8_t *s, uint16_t size) {
	for(uint8_t i = 0; i < size; i++) {
		lcd_putc(hi2c, s[i]);
	}
}

void lcd_puti(I2C_HandleTypeDef *hi2c, int num) {
	writeByte(hi2c, DR, (uint8_t)(num + 0x30));
}
