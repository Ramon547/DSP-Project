/*
 * EEPROM_Driver.c
 *
 *  Created on: May 3, 2025
 *      Author: Paul Janes
 */

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "EEPROM_Driver.h"

//Main functions of EEPROMs, from datasheet
const uint8_t WRSR = 0x01;
const uint8_t WRITE = 0x02;
const uint8_t READ = 0x03;
const uint8_t RDSR = 0x05;
const uint8_t WREN = 0x06;
const uint8_t STATUS = 0x00;

//Function to send write enable command to one specified module
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write_Enable_Mod(SPI_HandleTypeDef* SPI, uint8_t modSel){

	//Parse modSel for bank bit and two hold-select bits
	uint8_t bank = (modSel & 0x04) >> 2;
	uint8_t hold1 = (modSel & 0x02) >> 1;
	uint8_t hold0 = modSel & 0x01;

	//Initialize return value
	HAL_StatusTypeDef retVal = 0;

	//Set hold-select pins to correct value
	HAL_GPIO_WritePin(GPIOA, HOLD_HI, hold1);
	HAL_GPIO_WritePin(GPIOA, HOLD_LO, hold0);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Send write enable and get status
	retVal = HAL_SPI_Transmit(SPI, &WREN, 1, 1);

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to send write enable to entire bank
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write_Enable_BC(SPI_HandleTypeDef* SPI, uint8_t modSel){

	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	HAL_StatusTypeDef retVal = 0;

	//Set broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Send write enable and get status
	retVal = HAL_SPI_Transmit(SPI, &WREN, 1, 1);

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to read status register of an EEPROM module
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Returns contents of status register
uint8_t EEPROM_Read_SR(SPI_HandleTypeDef* SPI, uint8_t modSel){

	//Parse modSel for bank bit and two hold-select bits
	uint8_t bank = (modSel & 0x04) >> 2;
	uint8_t hold1 = (modSel & 0x02) >> 1;
	uint8_t hold0 = modSel & 0x01;

	//Initialize return value
	uint8_t retVal = 0;

	//Set hold-select pins to correct value
	HAL_GPIO_WritePin(GPIOA, HOLD_HI, hold1);
	HAL_GPIO_WritePin(GPIOA, HOLD_LO, hold0);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Transmit read status register command and receive response
	HAL_SPI_Transmit(SPI, &RDSR, 1, 1);
	HAL_SPI_Receive(SPI, &retVal, 1, 1);

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status register contents
	return retVal;
}

//Function to read write in progress bit from status register
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Returns contents of WIP bit
uint8_t EEPROM_Read_WIP(SPI_HandleTypeDef* SPI, uint8_t modSel){

	//Read SR and return LSB, which is the WIP bit
	return EEPROM_Read_SR(SPI, modSel) & 0x01;
}

//Function to write to the status register of a single EEPROM module
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to new status register contents
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write_SR_Mod(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* SR){

	//Parse modSel for bank bit and two hold-select bits
	uint8_t bank = (modSel & 0x04) >> 2;
	uint8_t hold1 = (modSel & 0x02) >> 1;
	uint8_t hold0 = modSel & 0x01;

	//Initialize return value
	uint8_t retVal = 0;

	//Attempt write enable, return HAL status on failure
	if((retVal = EEPROM_Write_Enable_Mod(SPI, modSel)) != HAL_OK){
		return retVal;
	}

	//Set hold-select pins to correct value
	HAL_GPIO_WritePin(GPIOA, HOLD_HI, hold1);
	HAL_GPIO_WritePin(GPIOA, HOLD_LO, hold0);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Send write status register command and SR contents
	HAL_SPI_Transmit(SPI, &WRSR, 1, 1);
	retVal = HAL_SPI_Transmit(SPI, SR, 1, 1);

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to write to status register of a whole bank
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to new status register contents
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write_SR_BC(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* SR){

	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	uint8_t retVal = 0;

	//Attempt write enable, return HAL status on failure
	if((retVal = EEPROM_Write_Enable_BC(SPI, modSel)) != HAL_OK){
		return retVal;
	}

	//Pull broadcast pin HI, as it is left LO in write enable function
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Send write status register command and SR contents
	HAL_SPI_Transmit(SPI, &WRSR, 1, 1);
	retVal = HAL_SPI_Transmit(SPI, SR, 1, 1);

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to write 32-bit word across 4 EEPROM modules in 1 bank
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to 32-bit data, assuming it's stored as 4 8-bit data points in an array
//Takes in pointer to 24-bit address, assuming it's stored as 3 8-bit data points in an array
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t* address){

	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	uint8_t retVal = 0;

	//Attempt write enable and return status on failure
	if((retVal = EEPROM_Write_Enable_BC(SPI, modSel)) != HAL_OK){
		return retVal;
	}

	//Pull broadcast pin HI, as it is left LO in write enable function
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Broadcast write command and address
	HAL_SPI_Transmit(SPI, &WRITE, 1, 1);
	HAL_SPI_Transmit(SPI, address, 3, 1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//For loop to write to each of 4 modules
	for(uint8_t i = 0; i < 4; i++){
		//Set hold-select pins based on iterand
		HAL_GPIO_WritePin(GPIOA, HOLD_LO, i & 0x01);
		HAL_GPIO_WritePin(GPIOA, HOLD_HI, i & 0x02);

		//Attempt to send data[i] to module i, return status on failure
		if((retVal = HAL_SPI_Transmit(SPI, data + i, 1, 1)) != HAL_OK){
			return retVal;
		}
	}

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to read 32-bit word from 4 EEPROM modules in 1 bank
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to data, where it will store the read values
//Takes in pointer to 24-bit address, assuming it's stored as 3 8-bit data points in an array
//Returns HAL status
HAL_StatusTypeDef EEPROM_Read(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t* address){
	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	uint8_t retVal = 0;

	//Pull broadcast pin HI
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Broadcast read command and address
	HAL_SPI_Transmit(SPI, &READ, 1, 1);
	HAL_SPI_Transmit(SPI, address, 3, 1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//For loop to read from each of the 4 modules
	for(uint8_t i = 0; i < 4; i++){
		//Set hold-select pins based on iterand
		HAL_GPIO_WritePin(GPIOA, HOLD_LO, i & 0x01);
		HAL_GPIO_WritePin(GPIOA, HOLD_HI, i & 0x02);

		//Attempt to send data[i] to module i, return status on failure
		if((retVal = HAL_SPI_Receive(SPI, data + i, 1, 1)) != HAL_OK){
			return retVal;
		}
	}

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to write bulk data to EEPROMs
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to data, assuming it's stored in a 2D array of the form array[n][4], with each element being 8 bits of a 32-bit word
//Takes in number of points to write n
//Takes in pointer to 24-bit address, assuming it's stored as 3 8-bit data points in an array
//Returns HAL status
HAL_StatusTypeDef EEPROM_Write_Bulk(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t n, uint8_t* address){
	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	uint8_t retVal = 0;

	//Attempt write enable and return status on failure
	if((retVal = EEPROM_Write_Enable_BC(SPI, modSel)) != HAL_OK){
		return retVal;
	}

	//Pull broadcast pin HI, as it is left LO in write enable function
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Broadcast write command and address
	HAL_SPI_Transmit(SPI, &WRITE, 1, 1);
	HAL_SPI_Transmit(SPI, address, 3, 1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//Nested for loop to iterate through 2D data array
	for(uint8_t i = 0; i < 4; i++){

		//Set hold-select pins based on iterand
		HAL_GPIO_WritePin(GPIOA, HOLD_LO, i & 0x01);
		HAL_GPIO_WritePin(GPIOA, HOLD_HI, i & 0x02);

		//Inner loop to actually write, one full row at a time
		for(uint8_t j = i; j < n << 2; j += 4){

			//Attempt to send data[i] to module i, return status on failure
			if((retVal = HAL_SPI_Transmit(SPI, data + j, 1, 1)) != HAL_OK){
				return retVal;
			}
		}

		//Establish timeout for following while loop
		uint32_t timeout = HAL_GetTick() + EEPROM_DELAY;

		//While loop to wait for write to complete
		while(EEPROM_Read_WIP(SPI, modSel)){
			if(HAL_GetTick() > timeout){
				return HAL_TIMEOUT;
			}
		}
	}

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}

//Function to read bulk data from EEPROMs
//Takes in a pointer to an SPI instance
//Takes in 3-bit module ID
//Bits 7:3 are don't care
//Takes in pointer to data, assuming it's stored in a 2D array of the form array[n][4], with each element being 8 bits of a 32-bit word
//Takes in number of points to write n
//Takes in pointer to 24-bit address, assuming it's stored as 3 8-bit data points in an array
//Returns HAL status
HAL_StatusTypeDef EEPROM_Read_Bulk(SPI_HandleTypeDef* SPI, uint8_t modSel, uint8_t* data, uint8_t n, uint8_t* address){
	//Parse modSel for bank bit
	uint8_t bank = (modSel & 0x04) >> 2;

	//Initialize return value
	uint8_t retVal = 0;

	//Pull broadcast pin HI
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_SET);

	//Toggle CS for correct bank, signaling incoming command
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Broadcast read command and address
	HAL_SPI_Transmit(SPI, &READ, 1, 1);
	HAL_SPI_Transmit(SPI, address, 3, 1);

	//Clear broadcast pin
	HAL_GPIO_WritePin(GPIOA, BC, GPIO_PIN_RESET);

	//For loop to read from each of the 4 modules
	for(uint8_t i = 0; i < 4; i++){

		//Set hold-select pins based on iterand
		HAL_GPIO_WritePin(GPIOA, HOLD_LO, i & 0x01);
		HAL_GPIO_WritePin(GPIOA, HOLD_HI, i & 0x02);

		//For loop to read each data point
		for(uint8_t j = i; j < n << 2; j += 4){

			//Attempt to receive data[i] from module i, return status on failure
			if((retVal = HAL_SPI_Receive(SPI, data + j, 1, 1)) != HAL_OK){
				return retVal;
			}
		}
	}

	//Toggle CS again to signal end of sequence
	HAL_GPIO_WritePin(GPIOA, CS, !bank);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, CS, bank);
	HAL_Delay(1);

	//Need 7ms max to complete command
	HAL_Delay(EEPROM_DELAY - 2);

	//Return status
	return retVal;
}
