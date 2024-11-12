#include <iostream>
#include <stdio.h>
#include <util.h>
#include <Arduino.h>

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num){
  static uint8_t temp[2]; // initialize
  temp[0] = (num >> 8) & 255;  // Extract the high byte
  temp[1] = num & 255;         // Extract the low byte
  return temp;
}

// See if we need anything more from original data ,if not use pass by reference
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte){
  uint16_t temp = (Hbyte << 8) | Lbyte; // bitshiftLeft by 8 OR with the lower byte, then put to 2 different variable
  return temp;
}

// Encode float or uint16 into arrays of uint8, Can we pass by reference ?
unsigned char* Encode_bytearray(float f) { 
    // Use memcpy to copy the bytes of the float into the array
    static uint8_t c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    // Copy to address of array , Copy from address of float , size of float: Now, c[0] to c[3] contain the bytes of the float
    return c; 
}

// Decode byte arrays into float or uint16 : This one is pass by pointer
float Decode_bytearray(unsigned char* c) {
    float f;
    // Use memcpy to copy the bytes from the array back into the float
    memcpy(&f, c, sizeof(f));
    return f;
}

// Split and Check bit from MSB -> LSB
unsigned char* checkstatMSB(unsigned char num){
  static uint8_t arr[8]; // array to hold 8 binary number
    for (int i = 7; i >= 0; i--){
        arr[7-i] = (num >> i) & 1;
        // 1st shift will shift to right by 7 position
        // Ex 1 => 0b00000001 , then AND with 1 so anything that isn't one at 1st pos will be cut off
        // 00101010 >> 7 = 00000000 & 00000001 = 0 (point at 1st bit 1 encounter from left . shift to right by 7 pos)
        // 00101010 >> 6 = 00000000 & 00000001 = 0
        // 00101010 >> 5 = 00000001 & 00000001 = 1
        // Doing arr[7-i] because index order is 0->nth not the other way around, in which it conflit a bit with MSB reading
    }    

  return arr;
}

void checkstatLSB(SDCstatus* STAT, unsigned char num){
  // static uint8_t arr[8]; // array to hold 8 binary number
  // STAT->shutdownsig = 1;
  for (int i = 0; i < 8; i++){
    uint8_t bit = num & 1;
    STAT->statbin[i] = bit;
    if(bit == 1){
        STAT->shutdownsig = 0;
    }
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again

    // 1st shift will shift to right by 7 position
        // 1 => 0b00000001 , then AND with 1 so anything that isn't one at 1st pos will be cut off
        // Ex. 42 = 0b00101010
        // 00101010 >> 0 = 00101010 & 00000001 = 0 (point at 1st bit 1 encounter from left . shift to right by 0 pos)
        // 00101010 >> 1 = 00010101 & 00000001 = 1
        // 00101010 >> 2 = 00001010 & 00000001 = 0
    // Check for bit 1 for immediate shutdown
  } 
}