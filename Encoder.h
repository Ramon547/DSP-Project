/*
 * Encoder.h
 *
 *  Created on: May 8, 2025
 *      Author: ramon
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include "stm32h7xx_hal.h"

// Initialize both encoders
void Encoder_Init(void);

// Get the current count value from Encoder 1 (TIM8)
int32_t Encoder1_GetCount(void);

// Get the current count value from Encoder 2 (TIM2)
int32_t Encoder2_GetCount(void);

// Reset encoder counts
void Encoder1_Reset(void);
void Encoder2_Reset(void);

// Encoder button callback hooks (to be filled by user)
void Encoder1_Button_Callback(void);
void Encoder2_Button_Callback(void);

#endif /* SRC_ENCODER_H_ */
