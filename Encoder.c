/*
 * Encoder.c
 *
 *  Created on: May 8, 2025
 *      Author: ramon
 */

#include "Encoder.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"  // timer functions


// External timer handles (make sure they're declared in main.c or tim.c)
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim2;

// Initialize both encoders
void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

// Get encoder count
int32_t Encoder1_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim8);
}

int32_t Encoder2_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}

// Reset encoder counters
void Encoder1_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim8, 0);
}

void Encoder2_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

// Button interrupt callbacks (these can be customized)
void Encoder1_Button_Callback(void) {
    // Example: toggle mode or trigger action
}

void Encoder2_Button_Callback(void) {
    // Example: toggle mode or trigger action
}
