/*
 * Codec_Driver.c
 *
 *  Created on: May 5, 2025
 *      Author: Paul Janes
 */

//Includes
#include <stdint.h>
#include "Data_Handler.h"
#include "stm32h7xx_hal.h"
#include "Codec_Driver.h"

const uint8_t REGS[] = {0x1F, 0x32, 0x35, 0x56, 0x58,
						0x45, 0x4B, 0x5C, 0x5A, 0x05,
						0x05, 0x07, 0x07, 0x08, 0x68,
						0x6A, 0x6C, 0x0A, 0x5E};

const uint8_t VALS[] = {0xFF, 0xFC, 0xE1, 0x51, 0x51,
						0x00, 0x00, 0x00, 0x00, 0x7F,
						0x7F, 0x7F, 0x7F, 0x05, 0x38,
						0x31, 0x26, 0x00, 0x0C};

//Function to send W32 formatted data to codec via I2S
//Takes in pointer to I2S handle
//Takes in pointer to W32 data
//Converts data to I2S format and sends to codec
//Returns HAL status
HAL_StatusTypeDef W32_Codec(I2S_HandleTypeDef* Codec, uint32_t* W32){

	//Create holder array for I2S data
	uint16_t I2S[3];

	//Convert W32 to I2S
	W32_I2S(W32, I2S, 1);

	//Transmit I2S data and return HAL status
	return HAL_I2S_Transmit(Codec, I2S, 3, CODEC_DELAY);
}

//Function to send PJ formatted data to codec via I2S
//Takes in pointer to I2S handle
//Takes in pointer to PJ formatted data
//Assumes data at pointer location is Left data, followed by Right data
//Converts data to I2S format and sends to codec
//Returns HAL status
HAL_StatusTypeDef PJ_Codec(I2S_HandleTypeDef* Codec, uint8_t* PJ){

	//Create holder array for I2S data
	uint16_t I2S[3];

	//Convert PJ to I2S
	PJ_I2S(PJ, I2S, 1);

	//Transmit I2S data and return HAL status
	return HAL_I2S_Transmit(Codec, I2S, 3, CODEC_DELAY);
}

//Function to send I2S formatted data to codec via I2S
//Takes in pointer to I2S handle
//Takes in pointer to I2S formatted data
//Assumes no conversion necessary
//Sends data straight through to codec
//Returns HAL status
HAL_StatusTypeDef I2S_Codec(I2S_HandleTypeDef* Codec, uint16_t* I2S){

	//Transmit I2S data and return HAL status
	return HAL_I2S_Transmit(Codec, I2S, 3, CODEC_DELAY);
}

//Function to receive W32 formatted data from codec
//Takes in pointer to I2S handle
//Takes in pointer to destination for W32 data
//Receives data from codec
//Converts to W32 and stores
//Returns HAL status
HAL_StatusTypeDef Codec_W32(I2S_HandleTypeDef* Codec, uint32_t* W32){

	//Create holder array for I2S data
	uint16_t I2S[3];

	//Instantiate return value
	HAL_StatusTypeDef retVal = 0;

	//Receive I2S information from codec, return status on failure
	if((retVal = HAL_I2S_Receive(Codec, I2S, 3, CODEC_DELAY)) != HAL_OK){
		return retVal;
	}

	//Convert I2S data to W32
	I2S_W32(W32, I2S, 1);

	//Return status
	return retVal;
}

//Function to receive PJ formatted data from codec via I2S
//Takes in pointer to I2S handle
//Takes in pointer to PJ formatted data destination
//Receives data from codec
//Converts to PJ and stores Left data, then Right data
//Returns HAL status
HAL_StatusTypeDef Codec_PJ(I2S_HandleTypeDef* Codec, uint8_t* PJ){

	//Create holder array for I2S data
	uint16_t I2S[3];

	//Instantiate return value
	HAL_StatusTypeDef retVal = 0;

	//Receive I2S information from codec, return status on failure
	if((retVal = HAL_I2S_Receive(Codec, I2S, 3, CODEC_DELAY)) != HAL_OK){
		return retVal;
	}

	//Convert I2S to PJ
	I2S_PJ(I2S, PJ, 1);

	//Return status
	return retVal;
}

//Function to receive I2S data from codec
//Takes in pointer to I2S handle
//Takes in pointer to I2S data destination
//Receives data from codec and stores
//Does no conversion
//Returns HAL status
HAL_StatusTypeDef Codec_I2S(I2S_HandleTypeDef* Codec, uint16_t* I2S){

	//Return status of attempting to receive I2S data
	return HAL_I2S_Receive(Codec, I2S, 3, CODEC_DELAY);
}

//Function to send W32 data to PWM output
//Takes in pointer to timer
//Takes in pointer to W32 data
//No output
void W32_PWM(TIM_HandleTypeDef* tim, uint32_t* W32){

	//Output left data as 10 MSBs of the 24 bits of data in the 32-bit left W32 element
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, ((W32[0] + 0x00800000) >> 14) & 1023);
	//Output right data as 10 MSBs of the 24 bits of data in the 32-bit right W32 element
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, ((W32[1] + 0x00800000) >> 14) & 1023);

}

//Function to send PJ data to PWM output
//Takes in pointer to timer
//Takes in pointer to PJ data
//No output
//Calls W32_PWM and converts
void PJ_PWM(TIM_HandleTypeDef* tim, uint8_t* PJ){

	uint32_t W32[2];

	PJ_W32(PJ, W32, 1);

	W32_PWM(tim, W32);
}

//Function to send I2S data to PWM output
//Takes in pointer to timer
//Takes in pointer to I2S data
//No output
//Calls W32_PWM and converts
void I2S_PWM(TIM_HandleTypeDef* tim, uint16_t* I2S){

	uint32_t W32[2];

	I2S_W32(W32, I2S, 1);

	W32_PWM(tim, W32);
}
