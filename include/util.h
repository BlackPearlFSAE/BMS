// --General Util

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);

// --Encode and Decode byte array
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);



// --Charging Shutdown & BMS Util

struct SDCstatus { // Need to change to more specific name
  uint8_t statbin[8];
  bool shutdownsig = 1; // Default should be 1 = OK , 0 = SHUTDOWN
};
void checkstatLSB(SDCstatus* STAT, unsigned char num);

// --BMS Specific Util


// --ESP32 Monitor board Util
#define STANDARD_DLC 8
#define STANDARD_BITRATE 250E3
struct _can_frame {
  uint32_t can_id;
  uint16_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};