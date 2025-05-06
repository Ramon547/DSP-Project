/*
 * DSP.h
 *
 *  Created on: May 5, 2025
 *      Author: Paul Janes
 */

#ifndef INC_DSP_H_
#define INC_DSP_H_

#include <stdint.h>
#include "Data_Handler.h"

//Q and max multiple for fixed-point division
#define Q			8
#define FIXED_MAX	0x00FFFFFF

//Function prototypes
void Volume_W32(uint32_t* W32, uint32_t n, uint8_t vol);
void Volume_PJ(uint8_t* PJ, uint32_t n, uint8_t vol);
void Volume_I2S(uint16_t* I2S, uint32_t n, uint8_t vol);

void Doubler_W32(uint32_t* W32, uint32_t n, uint8_t x);
void Doubler_PJ(uint8_t* PJ, uint32_t n, uint8_t x);
void Doubler_I2S(uint16_t* I2S, uint32_t n, uint8_t x);

void Add_Signals_W32(uint32_t* sig1, uint32_t* sig2, uint32_t n);
void Add_Signals_PJ(uint8_t* sig1, uint8_t* sig2, uint32_t n);
void Add_Signals_I2S(uint16_t* sig1, uint16_t* sig2, uint32_t n);

void Reverb_W32(uint32_t* now, uint32_t* past, uint32_t n, uint8_t x, uint8_t scale);
void Reverb_PJ(uint8_t* now, uint8_t* past, uint32_t n, uint8_t x, uint8_t scale);
void Reverb_I2S(uint16_t* now, uint16_t* past, uint32_t n, uint8_t x, uint8_t scale);

#endif /* INC_DSP_H_ */
