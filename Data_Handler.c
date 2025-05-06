/*
 * Data_Handler.c
 *
 *  Created on: May 4, 2025
 *      Author: Paul Janes
 */

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "Data_Handler.h"

//Function to convert I2S data to 4-byte EEPROM-friendly format
//Takes in I2S data as an array, assumed to be of the form uint16_t* I2S[n][3]
//Takes in the destination for the packed data, assumed to be of the form uint8_t* PJ[2*n][4]
//The data will be stored such that left and right audio points are adjacent in the output array
//No return value
void I2S_PJ(uint16_t* I2S, uint8_t* PJ, uint32_t n){

	//m is data points in PJ
	//m = 8n because PJ has two data points per I2S data point and 4 elements per data point
	uint8_t m = n << 3;

	//n = 3n because there are 3 elements per data point in I2S
	n = n + (n << 1);

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n - 2 && j < m - 7; i += 3, j += 8){

		//Assign left data to PJ format
		//First byte is initialized to 0
		PJ[j] = 0x00;
		PJ[j + 1] = (uint8_t)((I2S[i] & 0xFF00) >> 8);
		PJ[j + 2] = (uint8_t)(I2S[i] & 0x00FF);
		PJ[j + 3] = (uint8_t)((I2S[i + 1] & 0xFF00) >> 8);

		//Assign right data to PJ format
		//First byte is initialized to 0
		PJ[j + 4] = 0x00;
		PJ[j + 5] = (uint8_t)(I2S[i + 1] & 0x00FF);
		PJ[j + 6] = (uint8_t)((I2S[i + 2] & 0xFF00) >> 8);
		PJ[j + 7] = (uint8_t)(I2S[i + 2] & 0x00FF);
	}
}

//Function to convert 4-byte EEPROM data into I2S format
//Takes in EEPROM data array, assumed to be of the form uint8_t* PJ[2 * n][4]
//Takes in I2S destination array, assumed to be of the form uint16_t* I2S[n][3]
//The data will be stored such that left and right adjacency is preserved
//Takes in size n
//No return value
void PJ_I2S(uint8_t* PJ, uint16_t* I2S, uint32_t n){

	//m is data points in PJ
	//m = 8n because PJ has two data points per I2S data point and 4 elements per data point
	uint32_t m = n << 3;

	//n = 3n because there are 3 elements per data point in I2S
	n = n + (n << 1);

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n && j < m; i += 3, j += 8){

		//PJ[x][0] is ignored here
		//PJ[x][1] becomes the first byte of I2S[i]
		//PJ[x][2] becomes the second byte
		I2S[i] = ((uint16_t) PJ[j + 1]) << 8;
		I2S[i] = I2S[i] | (uint16_t) PJ[j + 2];

		//PJ[x][4] is ignored here
		//PJ[x][3] becomes the first byte of I2S[i + 1]
		//PJ[x][5] becomes the second byte
		I2S[i + 1] = ((uint16_t) PJ[j + 3]) << 8;
		I2S[i + 1] = I2S[i] | (uint16_t) PJ[j + 5];

		//PJ[x][6] becomes the first byte of I2S[i] + 2
		//PJ[x][7] becomes the second byte
		I2S[i + 2] = ((uint16_t) PJ[j + 6]) << 8;
		I2S[i + 2] = I2S[i] | (uint16_t) PJ[j + 7];
	}
}

//Function to convert PJ data to 32-bit words
//Takes in pointer to PJ data
//Takes in pointer to W32 data
//Assumes W32 L and R data are adjacent, as in PJ data
//Takes in number of points n
//n should be total points, L and R, to convert
//No return value
void PJ_W32(uint8_t* PJ, uint32_t* W32, uint32_t n){

	//Calculate size m, since one PJ data point is 4 elements
	uint32_t m = n << 2;

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n && j < m - 3; i++, j += 4){
		W32[i] = (((uint32_t) PJ[j]) << 24) | (((uint32_t) PJ[j + 1]) << 16) | (((uint32_t) PJ[j + 2]) << 8) | (uint32_t) PJ[j + 3];
	}
}

//Function to convert 32-bit words into PJ data format
//Takes in pointer to 32-bit words
//Takes in pointer to PJ data array
//Takes in number of data points to process n
//No return value
void W32_PJ(uint32_t* W32, uint8_t* PJ, uint32_t n){

	//Calculate size m, since one PJ data point is 4 elements
	uint32_t m = n << 2;

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n && j < m - 3; i++, j += 4){
		//MSB -> PJ[0] ... LSB -> PJ[3]
		PJ[j] = (uint8_t) ((W32[i] & 0xFF000000) >> 24);
		PJ[j + 1] = (uint8_t) ((W32[i] & 0x00FF0000) >> 16);
		PJ[j + 2] = (uint8_t) ((W32[i] & 0x0000FF00) >> 8);
		PJ[j + 3] = (uint8_t) (W32[i] & 0x000000FF);
	}
}

//Function to convert 32-bit word into I2S format
//Takes in pointer to W32 data
//Takes in pointer to I2S data destination
//Takes in number of data points n
//No return value
void W32_I2S(uint32_t* W32, uint16_t* I2S, uint32_t n){

	//Calculate size m, since one I2S splits to two W32
	uint32_t m = n << 1;

	//Rescale n to reflect 3 elements per I2S data point
	n = m + n;

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n - 2 && j < m - 1; i += 3, j += 2){

		//First 16-bit word is just bytes 2 and 1 of the left data
		I2S[i] = (uint16_t) ((W32[j] & 0x00FFFF00) >> 8);

		//Second 16-bit word is the LSB of left data as MSB and byte 2 of right data as LSB
		I2S[i + 1] = (uint16_t) ((W32[j] & 0x000000FF) << 8) | (uint16_t) ((W32[j + 1] & 0x00FF0000) >> 16);

		//Third 16-bit word is just the two LSB of the right data
		I2S[i + 2] = (uint16_t) (W32[j + 1] & 0x0000FFFF);
	}
}

//Function to convert I2S data into W32 format
//Takes in pointer to I2S data
//Takes in pointer to W32 destination
//Takes in number of points n
//No return value
void I2S_W32(uint32_t* W32, uint16_t* I2S, uint32_t n){

	//Calculate size m, since one I2S splits to two W32
	uint32_t m = n << 1;

	//Rescale n to reflect 3 elements per I2S data point
	n = m + n;

	//For loop to iterate through all data points
	for(uint8_t i = 0, j = 0; i < n - 2 && j < m - 1; i += 3, j += 2){

		//Left W32 is 0x00 followed by the first 16-bit word, followed by the MSB of the second 16-bit word
		W32[j] = (((uint32_t) I2S[i]) << 8) | (((uint32_t) (I2S[i + 1] & 0xFF00)) >> 8);

		//Right W32 is 0x00 followed by LSB of second 16-bit word, followed by the third 16-bit word
		W32[j + 1] = (((uint32_t) (I2S[i + 1] & 0x00FF)) << 16) | ((uint32_t) I2S[i + 2]);
	}
}

//Function to convert one byte to four chars representing hex digits
//Takes in the byte
//Takes in a pointer to the hex string destination
//Will store the hex string at the pointer
//No return value
void Hexify(uint8_t byte, uint8_t* hex){

	//Switch...case for first char
	switch((byte >> 4) & 0x0F){
		case 0:
			hex[0] = '0';
			break;
		case 1:
			hex[0] = '1';
			break;
		case 2:
			hex[0] = '2';
			break;
		case 3:
			hex[0] = '3';
			break;
		case 4:
			hex[0] = '4';
			break;
		case 5:
			hex[0] = '5';
			break;
		case 6:
			hex[0] = '6';
			break;
		case 7:
			hex[0] = '7';
			break;
		case 8:
			hex[0] = '8';
			break;
		case 9:
			hex[0] = '9';
			break;
		case 10:
			hex[0] = 'A';
			break;
		case 11:
			hex[0] = 'B';
			break;
		case 12:
			hex[0] = 'C';
			break;
		case 13:
			hex[0] = 'D';
			break;
		case 14:
			hex[0] = 'E';
			break;
		case 15:
			hex[0] = 'F';
			break;
		default:
			hex[0] = '\0';
			break;
	}

	//Switch..case for second char
	switch(byte & 0x0F){
		case 0:
			hex[1] = '0';
			break;
		case 1:
			hex[1] = '1';
			break;
		case 2:
			hex[1] = '2';
			break;
		case 3:
			hex[1] = '3';
			break;
		case 4:
			hex[1] = '4';
			break;
		case 5:
			hex[1] = '5';
			break;
		case 6:
			hex[1] = '6';
			break;
		case 7:
			hex[1] = '7';
			break;
		case 8:
			hex[1] = '8';
			break;
		case 9:
			hex[1] = '9';
			break;
		case 10:
			hex[1] = 'A';
			break;
		case 11:
			hex[1] = 'B';
			break;
		case 12:
			hex[1] = 'C';
			break;
		case 13:
			hex[1] = 'D';
			break;
		case 14:
			hex[1] = 'E';
			break;
		case 15:
			hex[1] = 'F';
			break;
		default:
			hex[1] = '\0';
			break;
	}

}

//Function to convert one byte to 8 chars representing binary digits
//Takes in the byte
//Takes in a pointer to the bin string destination
//Will store the bin string at the pointer
//No return value
void Binify(uint8_t byte, uint8_t* bin){

	//For loop to iterate through each bit
	for(uint8_t i = 0; i < 8; i++){

		//If bit i = 1, char i = '1' + 0
		//If bit i = 0, char i = 0 + '0'
		bin[i] = '1' * ((byte & 0x80) >> i) + '0' * !((byte & 0x80) >> i);
	}

}

//Function to display data via UART
//Takes in a pointer to the UART handle
//Takes in a pointer to the data to be displayed
//Takes in the number of data points
//Takes in a format identifier between 0 and 2
//Takes in a pointer to a location to hold the results of the formatting
//Inserts new line char after each byte is processed
//Will consider format to be 2 for all 0bxxxxxx1x
//Will consider format to be 1 for all 0bxxxxxx01
//Will consider format to be 0 for all 0bxxxxxx00
//Just use the defined constants in the corresponding header file
//No return value
void Display_Serial(UART_HandleTypeDef* UART, uint8_t* data, uint16_t n, uint8_t format, uint8_t* holder){

	//New length of actual data, default n
	uint16_t m = n << 1;

	//Switch...case to recalculate total length from n and format
	switch(format & 0x03){
		case 3:
		case 2:
			//If format is 2 or 3, convert to binary
			m = (m << 2) + n;
			//For loop to convert to binary
			for(uint16_t i = 0, j = 0; i < n && j < m; i++, j += 9){
				//Each data point needs 8 chars to represent it
				Binify(data[i], holder + j);
				holder[j + 8] = newLine;
			}
			break;
		case 1:
			//If format is 1, convert to hex
			m = m + n;
			//For loop to convert to hex
			for(uint16_t i = 0, j = 0; i < n && j < m; i++, j += 3){
				//Each data point needs 2 chars to represent it
				Hexify(data[i], holder + j);
				holder[j + 2] = newLine;
			}
			break;
		default:
			//For loop to transfer data directly to holder
			for(uint16_t i = 0; i < m; i++){
				//holder = data
				holder[i] = data[i];
				holder[++i] = newLine;
			}
			break;
	}

	//Send data to serial connection
	HAL_UART_Transmit(UART, holder, m, m);
}
