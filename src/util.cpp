#include <iostream>
#include <stdio.h>
#include <cstring>
#include <util.h>
#include <Arduino.h>
// #include <ArduinoSTL.h>


// enum SDCSTATUS{
//   OK = 2,
//   WARNING = 1,
//   NOT_OK = 0

// };


/*------------------------------ General Functions ----*/
// Split uint16_t to High byte and Low byte
unsigned char* splitHLbyte(unsigned int num){
  static uint8_t temp[2]; // initialize
  temp[0] = (num >> 8) & 255;  // Extract the high byte
  temp[1] = num & 255;         // Extract the low byte
  return temp;
}

// Merge 2 bytes into uint16_t
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte){
  // ** May pass reference , think later
  uint16_t temp = (Hbyte << 8) | Lbyte; // bitshiftLeft by 8 OR with the lower byte, then put to 2 different variable
  return temp;
}

// Convert float or uint16 into byte arrays
unsigned char* Encode_bytearray(float f) { 
    static uint8_t c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    // Copy to address of array , Copy from address of float , size of float: Now, c[0] to c[3] contain the bytes of the float
    return c; 
}

// Convert byte arrays into float or uint16
float Decode_bytearray(unsigned char* c) {
    float f;
    // Use memcpy to copy the bytes from the array back into the float
    memcpy(&f, c, sizeof(f));
    return f;
}

// Convert Binary to Binary digit array
// Split and Check bit from MSB -> LSB
// 1st shift will shift to right by 7 position
        // 1 => 0b00000001 , then AND with 1 so anything that isn't one at 1st pos will be cut off
        // Ex. 42 = 0b00101010
        // 00101010 >> 0 = 00101010 & 00000001 = 0 (point at 1st bit 1 encounter from left . shift to right by 0 pos)
        // 00101010 >> 1 = 00010101 & 00000001 = 1
        // 00101010 >> 2 = 00001010 & 00000001 = 0
    // Check for bit 1 for immediate shutdown


// Split check and convert to bitarray with respect to MSB
uint16_t *toBitarrayMSB(unsigned char num){
  static uint16_t bitarr[8]; // array to hold 8 binary number
  for (int i = 7; i >= 0; i--){
    uint8_t bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// Split check and convert to bitarray with respect to LSB
uint16_t *toBitarrayLSB(unsigned char num){
  static uint16_t bitarr[8]; // array to hold 8 binary number
  for (int i = 0; i < 8; i++){
    uint8_t bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// /*------------------------------ Shutdown Circuit Functions ----*/
// void checkstatMSB(SDCstatus* STAT, unsigned char num){
//   // static uint8_t arr[8]; // array to hold 8 binary number
//   // STAT->shutdownsig = 1;
//   for (int i = 7; i >= 0; i--){
//     uint8_t bit = num & 1;
//     STAT->statbin[i] = bit;

//     // This is for immediate shutdown , might change
//     if(bit == 1){
//         STAT->SHUTDOWN_OK = 0;
//     }
//     num >>= 1; 
//   } 
// }

// void checkstatLSB(SDCstatus* STAT, unsigned char num){
//   // static uint8_t arr[8]; // array to hold 8 binary number
//   // STAT->shutdownsig = 1;
//   for (int i = 0; i < 8; i++){
//     uint8_t bit = num & 1;
//     STAT->statbin[i] = bit;

//     // This is for immediate shutdown , might change
//     if(bit == 1){
//         STAT->SHUTDOWN_OK = 0;
//     }
//     num >>= 1;
//   } 
// }



/*------------------------------ CAN comminication Functions ----
-----(Check the spreadsheet in README.md for CAN ID custom rules)*/

// 1 CE 1 0A 00
// 0001 1100 1110 0001 0000 1010 0000 0000

// Create Extended CAN ID of Priority , BaseID , msg Number , src , dest 
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t BASE_ID, uint8_t MSG_NUM ,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS) {
    uint32_t canID = 0;
    canID |= ((PRIORITY & 0x1F) << 28);        // (X)Priority (bits 29)
    canID |= ((BASE_ID & 0xFF) << 20);         // (BB) Base id denotes CAN channel (bit 28-20)
    canID |= ((MSG_NUM & 0xFF) << 16);         // (X) Message number (bits 19-16)
    canID |= ((SRC_ADDRESS & 0xFF) << 8);      // (SS)Source BMU address (bits 15-8)
    canID |= DEST_ADDRESS;                     // (DD)Destination BCU address (bits 7-0)
    return canID;
}

// Decode extended CAN ID
void decodeExtendedCANID(struct CANIDDecoded* myCAN ,uint32_t canID) {
    
    myCAN->PRIORITY = (canID >> 28) & 0x1F;        // Extract priority (bits 29)
    myCAN->BASE_ID = (canID >> 20) & 0xFF;         // Extract Message Number (bits 28-20)
    myCAN->MSG_NUM = (canID >> 16) & 0x0F;         // Extract Message Number (bits 19-16)
    myCAN->SRC = (canID >> 8) & 0xFF;              // Extract Source Address (bits 15-8)
    myCAN->DEST = canID & 0xFF;                    // Extract destination address (bits 7-0)
    
}


// BMS data structure