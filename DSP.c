/*
 * DSP.c
 *
 *  Created on: May 5, 2025
 *      Author: Paul Janes
 */
#include <stdint.h>
#include "DSP.h"
#include "Data_Handler.h"

//Function to alter the volume of a signal in W32 format
//Takes in a pointer to the signal data
//Takes in the size of the data n
//Takes in the volume level
//Max vol is 2^Q - 1
//Uses fixed-point division to multiply by at most 255/256
//Alters input in place
//No return value
void Volume_W32(uint32_t* W32, uint32_t n, uint8_t vol){

	//Two W32s per data point
	n = n << 1;

	//For loop to iterate through all data points
	for(uint32_t i = 0; i < n; i++){

		//Multiply by vol and divide by 2^Q
		W32[i] = (W32[i] * (uint32_t) vol) >> Q;
	}
}

//Function to alter the volume of a signal in PJ format
//Takes in a pointer to the signal data
//Takes in the size of the data n
//Takes in the volume level
//Just converts to W32 and calls Volume_32
//No return value
void Volume_PJ(uint8_t* PJ, uint32_t n, uint8_t vol){

	//PJ format has four times as many elements as W32
	uint32_t W32[n >> 2];

	//Convert PJ to W32
	PJ_W32(PJ, W32, n);

	//Perform volume adjustment on W32 data
	Volume_W32(W32, n, vol);

	//Convert back to PJ
	W32_PJ(W32, PJ, n);

}

//Function to alter the volume of a signal in I2S format
//Takes in a pointer to the signal data
//Takes in the size of the data n
//Takes in the volume level
//Just converts to W32 and calls Volume_32
//No return value
void Volume_I2S(uint16_t* I2S, uint32_t n, uint8_t vol){

	//Each I2S data point has 3 elements, vs 2 for W32
	uint32_t W32[(n + (n << 1)) >> 1];

	//Convert I2S to W32
	I2S_W32(W32, I2S, n);

	//Perform volume adjustment on W32 data
	Volume_W32(W32, n, vol);

	//Convert back to I2S
	W32_I2S(W32, I2S, n);
}

//Function to double the amplitude of a W32 signal a number of times
//Takes in a pointer to the signal data
//Takes in data size n
//Takes in number of times to double x
//Will preserve sign bit
//Will not allow overflow
//Alters data in place
//No return value
void Doubler_W32(uint32_t* W32, uint32_t n, uint8_t x){

	//"Boolean" to check if it's ok to shift, and an iterand
	uint8_t safe = 1, j = 0;

	//While loop to exit for safety and to track iterand up to x
	while(safe && j < x){

		//For loop to iterate through all data points
		for(uint32_t i = 0; i < (n << 1); i++){

			//First checks for safety
			//Safe to shift if MSb and 2nd MSb are the same
			safe = !(((W32[i] & 0x80000000) >> 31) ^ ((W32[i] & 0x40000000) >> 30));

			//Then shifts and fills in the right with whatever is on the left
			//Probably could have rotated, but I don't remember how
			//This is fine
			W32[i] = (W32[i] << safe) | (((W32[i] & 0x80000000) >> 31) & safe);

			//If not safe break
			//We don't like branching conditionals
			i = ((n << 1) & -!((uint32_t) safe)) | (i & -((uint32_t) safe));
		}

		//Increment iterand
		j++;
	}

}

//Function to double the amplitude of a PJ signal a number of times
//Takes in a pointer to the signal data
//Takes in data size n
//Takes in number of times to double x
//Will preserve sign bit
//Will not allow overflow
//Alters data in place
//Converts to W32 and calls Doubler_W32
//No return value
void Doubler_PJ(uint8_t* PJ, uint32_t n, uint8_t x){

	//PJ format has four times as many elements as W32
	uint32_t W32[n >> 2];

	//Convert PJ to W32
	PJ_W32(PJ, W32, n);

	//Perform doubling on W32 data
	Doubler_W32(W32, n, x);

	//Convert back to PJ
	W32_PJ(W32, PJ, n);
}

//Function to double the amplitude of a I2S signal a number of times
//Takes in a pointer to the signal data
//Takes in data size n
//Takes in number of times to double x
//Will preserve sign bit
//Will not allow overflow
//Alters data in place
//Converts to W32 and calls Doubler_W32
//No return value
void Doubler_I2S(uint16_t* I2S, uint32_t n, uint8_t x){

	//Each I2S data point has 3 elements, vs 2 for W32
	uint32_t W32[(n + (n << 1)) >> 1];

	//Convert I2S to W32
	I2S_W32(W32, I2S, n);

	//Perform doubling on W32 data
	Doubler_W32(W32, n, x);

	//Convert back to I2S
	W32_I2S(W32, I2S, n);
}

//Function to add two W32 signals together
//Takes in pointers to addend signals
//Takes in data size n
//Performs addition such that sig1 += sig2, altering sig1 in place and preserving sig2
//Preserves any metadata stored in bits 31..23
//No return value
void Add_Signals_W32(uint32_t* sig1, uint32_t* sig2, uint32_t n){

	//Placeholder to check for overflow
	uint32_t OFbit = 0x00000000;

	//For loop to iterate through all data points
	for(uint32_t i = 0; i < n << 1; i++){

		//Save bit 24 of sig1[i]
		OFbit = 0x01000000 & sig1[i];

		//Add signals
		sig1[i] += sig2[i];

		//Subtract off overflow, if any
		sig1[i] -= (sig1[i] & 0x01000000) ^ OFbit;
	}
}

//Function to add two PJ signals together
//Takes in pointers to addend signals
//Takes in data size n
//Performs addition such that sig1 += sig2, altering sig1 in place and preserving sig2
//Converts to W32 and calls Add_Signals_W32
//No return value
void Add_Signals_PJ(uint8_t* sig1, uint8_t* sig2, uint32_t n){

	//PJ format has four times as many elements as W32
	uint32_t W321[n >> 2];
	uint32_t W322[n >> 2];

	//Convert PJ to W32
	PJ_W32(sig1, W321, n);
	PJ_W32(sig2, W322, n);

	//Perform doubling on W32 data
	Add_Signals_W32(W321, W322, n);

	//Convert back to PJ
	W32_PJ(W321, sig1, n);
	W32_PJ(W322, sig2, n);
}

//Function to add two I2S signals together
//Takes in pointers to addend signals
//Takes in data size n
//Performs addition such that sig1 += sig2, altering sig1 in place and preserving sig2
//Converts to W32 and calls Add_Signals_W32
//No return value
void Add_Signals_I2S(uint16_t* sig1, uint16_t* sig2, uint32_t n){

	//Each I2S data point has 3 elements, vs 2 for W32
	uint32_t W321[(n + (n << 1)) >> 1];
	uint32_t W322[(n + (n << 1)) >> 1];

	//Convert I2S to W32
	I2S_W32(W321, sig1, n);
	I2S_W32(W322, sig2, n);

	//Perform doubling on W32 data
	Add_Signals_W32(W321, W322, n);

	//Convert back to I2S
	W32_I2S(W321, sig1, n);
	W32_I2S(W322, sig2, n);
}

//Function to apply reverb effect to a W32 signal
//Takes in pointer to receiving data point
//Takes in pointer to past data points
//Takes in size of past data n
//Takes in number of echoes to apply x
//Takes in scaling factor
//Scaling factor is relative strength of first echo
//Scaling factor decreases as additional echos are added

/*
 * THE PARAMETER NOW MUST BE A SIGNLE DATA POINT
 */

//Adds weaker and weaker pieces of the past offset more and more to create reverb effect
//Alters input now in place
//No return value
void Reverb_W32(uint32_t* now, uint32_t* past, uint32_t n, uint8_t x, uint8_t scale){

	//W32 stores two elements per data point
	uint32_t m = n << 1;

	//Store scale value to alter later
	uint8_t scale_this = scale;

	//Get scale reduction for successive echoes
	uint8_t scale_ea = scale / x;

	//Instantiate main signal strength
	uint8_t strength;

	//Determine number of data points between echoes
	//Always even because m = 2n
	uint32_t distance = m / x;

	//For loop to gather echoes
	for(uint32_t i = 0; i < m; i += distance){

		//Recalculate main signal strength
		strength = 0xFF - scale_this;

		//Adjust main signal to reduced strength
		Volume_W32(now, 1, strength);

		//Adjust echo to reduced strength
		Volume_W32(past + i, 1, scale_this);

		//Add echo to main signal
		Add_Signals_W32(now, past + i, 1);

		//Alter strength of next echo, accounting for underflow
		scale_this -= (-(scale_ea < scale_this)) & scale_ea;
	}
}

//Function to apply reverb effect to a PJ signal
//Takes in pointer to receiving data point
//Takes in pointer to past data points
//Takes in size of past data n
//Takes in number of echoes to apply x
//Takes in scaling factor for each successive echo

/*
 * THE PARAMETER NOW MUST BE A SIGNLE DATA POINT
 */

//Adds weaker and weaker pieces of the past offset more and more to create reverb effect
//Alters input now in place
//No return value
void Reverb_PJ(uint8_t* now, uint8_t* past, uint32_t n, uint8_t x, uint8_t scale){

	//PJ stores four elements per data point
	uint32_t m = n << 2;

	//Store scale value to alter later
	uint8_t scale_this = scale;

	//Get scale reduction for successive echoes
	uint8_t scale_ea = scale / x;

	//Instantiate main signal strength
	uint8_t strength;

	//Determine number of data points between echoes
	//Always divisible by 4 because m = 4n
	uint32_t distance = m / x;

	//For loop to gather echoes
	for(uint32_t i = 0; i < m; i += distance){

		//Recalculate main signal strength
		strength = 0xFF - scale_this;

		//Adjust main signal to reduced strength
		Volume_PJ(now, 1, strength);

		//Adjust echo to reduced strength
		Volume_PJ(past + i, 1, scale_this);

		//Add echo to main signal
		Add_Signals_PJ(now, past + i, 1);

		//Alter strength of next echo, accounting for underflow
		scale_this -= (-(scale_ea < scale_this)) & scale_ea;
	}
}

//Function to apply reverb effect to a I2S signal
//Takes in pointer to receiving data point
//Takes in pointer to past data points
//Takes in size of past data n
//Takes in number of echoes to apply x
//Takes in scaling factor for each successive echo

/*
 * THE PARAMETER NOW MUST BE A SIGNLE DATA POINT
 */

//Adds weaker and weaker pieces of the past offset more and more to create reverb effect
//Alters input now in place
//No return value
void Reverb_I2S(uint16_t* now, uint16_t* past, uint32_t n, uint8_t x, uint8_t scale){

	//I2S stores three elements per data point
	uint32_t m = n + (n << 1);

	//Store scale value to alter later
	uint8_t scale_this = scale;

	//Get scale reduction for successive echoes
	uint8_t scale_ea = scale / x;

	//Instantiate main signal strength
	uint8_t strength;

	//Determine number of data points between echoes
	//Always divisible by 3 because m = 3n
	uint32_t distance = m / x;

	//For loop to gather echoes
	for(uint32_t i = 0; i < m; i += distance){

		//Recalculate main signal strength
		strength = 0xFF - scale_this;

		//Adjust main signal to reduced strength
		Volume_I2S(now, 1, strength);

		//Adjust echo to reduced strength
		Volume_I2S(past + i, 1, scale_this);

		//Add echo to main signal
		Add_Signals_I2S(now, past + i, 1);

		//Alter strength of next echo, accounting for underflow
		scale_this -= (-(scale_ea < scale_this)) & scale_ea;
	}
}
