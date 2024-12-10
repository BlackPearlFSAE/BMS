#include <util.h>

#define STANDARD_BITRATE 500E3
// #define CAN_TIMEOUT_DURATION 4000; // Default setting is 4 second

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
struct CANIDDecoded {
    uint8_t PRIORITY;
    uint8_t BASE_ID;
    uint8_t MSG_NUM;
    uint8_t SRC;
    uint8_t DEST;
};

bool CANsend(_can_frame *sendPacket, void (*prepare)(_can_frame*));
bool CANsend(_can_frame *sendPacket);
bool CANreceive(_can_frame *receivePacket, void (*interpret)(_can_frame*));
bool CANreceive(_can_frame *receivePacket);

// Creating CAN ID :: Transmitter side
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t BASE_ID, uint8_t MSG_NUM ,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS);
// Structure of CAN ID :: Receiver side
void decodeExtendedCANID(struct CANIDDecoded* CANIDDecoded ,uint32_t canID);

