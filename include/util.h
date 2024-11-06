// --General Util

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);
// Split and Check bit from MSB -> LSB
// unsigned char *checkstatMSB(unsigned char num);
// Split and Check bit from LSB -> MSB
// unsigned char *checkstatLSB(unsigned char num);


// --Charging Shutdown & BMS Util

struct status { // Need to change to more specific name
  uint8_t statbin[8];
  bool shutdownsig = 1; // Default should be 1 = OK , 0 = SHUTDOWN
};
void checkstatLSB(struct status* STAT, unsigned char num);
// status checkstatLSB(struct status STAT, unsigned char num);

// --BMS Specific Util


// --ESP32 Monitor board Util