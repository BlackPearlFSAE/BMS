#include <iostream>
#include <stdio.h>
#include <cstring>

// --General Util

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);

// --Encode and Decode byte array
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);

// Return array of 16 binary digit from 16 bit Binary input
bool *toBitarrayMSB(uint16_t num);
bool *toBitarrayLSB(uint16_t num);

uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);


// --Shutdown mechanism & BMS specific Util