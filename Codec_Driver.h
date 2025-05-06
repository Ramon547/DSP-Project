/*
 * Codec_Driver.h
 *
 *  Created on: May 5, 2025
 *      Author: Paul Janes
 */

#ifndef SRC_CODEC_DRIVER_H_
#define SRC_CODEC_DRIVER_H_

//Includes
#include <stdint.h>
#include "Data_Handler.h"
#include "stm32h7xx_hal.h"

//Arrays holding registers and values to set to the registers
extern const uint8_t REGS[];
extern const uint8_t VALS[];

//Defines
#define CODEC_DELAY	10
#define ADDRESS		0x34
#define INIT_VALS	17

//Function prototypes
HAL_StatusTypeDef W32_Codec(I2S_HandleTypeDef* Codec, uint32_t* W32);
HAL_StatusTypeDef PJ_Codec(I2S_HandleTypeDef* Codec, uint8_t* PJ);
HAL_StatusTypeDef I2S_Codec(I2S_HandleTypeDef* Codec, uint16_t* I2S);
HAL_StatusTypeDef Codec_W32(I2S_HandleTypeDef* Codec, uint32_t* W32);
HAL_StatusTypeDef Codec_PJ(I2S_HandleTypeDef* Codec, uint8_t* PJ);
HAL_StatusTypeDef Codec_I2S(I2S_HandleTypeDef* Codec, uint16_t* I2S);

#endif /* SRC_CODEC_DRIVER_H_ */
