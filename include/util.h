// --General Util

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);

// --Encode and Decode byte array
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);



// --Shutdown & BMS Util

struct SDCstatus { // Need to change to more specific name
  uint8_t statbin[8];
  bool SHUTDOWN_OK_SIGNAL = 1; // Default should be 1 = OK , 0 = SHUTDOWN
  bool BMS_OK = 1;
  bool IMD_OK = 1;
  bool BSPD_OK = 1;
};
// void checkstatMSB(SDCstatus * STAT, unsigned char num);
// void checkstatLSB(SDCstatus* STAT, unsigned char num);
uint16_t *checkstatLSB(unsigned char num);
uint16_t *checkstatMSB(unsigned char num);

// --BMS Specific Util

// Creating CAN ID :: Transmitter side
uint32_t createExtendedCANID(uint8_t BASE_ID,uint8_t PRIORITY, uint8_t MSG_NUM ,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS);

// Structure of CAN ID :: Receiver side
struct CANIDDecoded {
    uint8_t PRIORITY;
    uint8_t BASE_ID;
    uint8_t MSG_NUM;
    uint8_t SRC;
    uint8_t DEST;
};

void decodeExtendedCANID(struct CANIDDecoded* CANIDDecoded ,uint32_t canID);


// --ESP32 Monitor board Util
#define STANDARD_DLC 8
#define STANDARD_BITRATE 500E3
struct _can_frame {
  uint32_t can_id;
  uint16_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));

  // Constructor function , to reset and initialize ID to nothing , and data frame to all 0
  _can_frame() 
        : can_id(0), can_dlc(0) {
        memset(data, 0, sizeof(data));
        } 
  

};