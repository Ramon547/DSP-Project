/*
 * EEPROM_Driver.h
 *
 *  Created on: May 3, 2025
 *      Author: Paul Janes
 */

#ifndef EEPROM_DRIVER_H_
#define EEPROM_DRIVER_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

//EEPROM Commands
extern const uint8_t WRSR;
extern const uint8_t WRITE;
extern const uint8_t READ;
extern const uint8_t RDSR;
extern const uint8_t WREN;
extern const uint8_t STATUS;

//Delay for EEPROM actions (max write time per datasheet)
#define EEPROM_DELAY	7

//Pin assignments for EEPROM access
#define CS              GPIO_PIN_9
#define BC              GPIO_PIN_8
#define HOLD_LO         GPIO_PIN_2
#define HOLD_HI         GPIO_PIN_3

//Function prototypes
HAL_StatusTypeDef EEPROM_Write_Enable_Mod(SPI_HandleTypeDef* SPI, uint8_t modSel);
HAL_StatusTypeDef EEPROM_Write_Enable_BC(SPI_HandleTypeDef* SPI, uint8_t modSel);
uint8_t EEPROM_Read_SR(SPI_HandleTypeDef* SPI, uint8_t modSel);
uint8_t EEPROM_Read_WIP(SPI_HandleTypeDef* SPI, uint8_t modSel);
HAL_StatusTypeDef EEPROM_Write_SR_Mod(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* SR);
HAL_StatusTypeDef EEPROM_Write_SR_BC(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* SR);
HAL_StatusTypeDef EEPROM_Write(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t* address);
HAL_StatusTypeDef EEPROM_Read(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t* address);
HAL_StatusTypeDef EEPROM_Write_Bulk(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t n, uint8_t* address);
HAL_StatusTypeDef EEPROM_Read_Bulk(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t n, uint8_t* address);

#endif //EEPROM_DRIVER_H_
