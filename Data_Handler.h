/*
 * Data_Handler.h
 *
 *  Created on: May 4, 2025
 *      Author: Paul Janes
 */

#ifndef INC_DATA_HANDLER_H_
#define INC_DATA_HANDLER_H_

//Includes
#include <stdint.h>
#include "stm32h7xx_hal.h"

//Some constant parameters for display functions
#define CHAR	0
#define HEX		1
#define BIN		2

//Define some data-related values that apply project-wide
#define I2S_POINTS			3
#define PJ_POINTS			4
#define W32_POINTS			2
#define SIZE_DECL			4096
#define SIZE_MINUS_ONE_DECL	4095

//Newline char constant
#define newLine '\n';

extern uint16_t SIZE;
extern uint16_t SIZE_MINUS_ONE;

//Function prototypes
void I2S_PJ(uint16_t* I2S, uint8_t* PJ, uint32_t n);
void PJ_I2S(uint8_t* PJ, uint16_t* I2S, uint32_t n);
void PJ_W32(uint8_t* PJ, uint32_t* W32, uint32_t n);
void W32_PJ(uint32_t* W32, uint8_t* PJ, uint32_t n);
void W32_I2S(uint32_t* W32, uint16_t* I2S, uint32_t n);
void I2S_W32(uint32_t* W32, uint16_t* I2S, uint32_t n);
void Hexify(uint8_t byte, uint8_t* hex);
void Binify(uint8_t byte, uint8_t* bin);
void Display_Serial(UART_HandleTypeDef* UART, uint8_t* data, uint16_t n, uint8_t format, uint8_t* holder);
void Table_Gen(CORDIC_HandleTypeDef* cordic);
void Table_Get_W32(uint16_t angle, uint32_t* value);
void Table_Get_PJ(uint16_t angle, uint8_t* value);

#endif /* INC_DATA_HANDLER_H_ */
