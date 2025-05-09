/*
 * Screen.h
 *
 *  Created on: May 7, 2025
 *      Author: ramon
 */

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define SCREEN_I2C_ADDR (0x3C << 1)  // Results in 0x78

void SCREEN_Init(I2C_HandleTypeDef *oled_i2c);
void SCREEN_WriteCommand(uint8_t cmd, I2C_HandleTypeDef *oled_i2c);
void SCREEN_WriteData(uint8_t *data, size_t size, I2C_HandleTypeDef *oled_i2c);
void SCREEN_SetCursor(uint8_t col, uint8_t page, I2C_HandleTypeDef *oled_i2c);
void SCREEN_WriteChar(char c, I2C_HandleTypeDef *oled_i2c, uint8_t col, uint8_t page);
void SCREEN_UpdateScreen(I2C_HandleTypeDef *oled_i2c);
void SCREEN_Clear(I2C_HandleTypeDef *oled_i2c);
void SCREEN_WriteString(const char *str, I2C_HandleTypeDef *oled_i2c, uint8_t col, uint8_t page);


#endif /* INC_SCREEN_H_ */
