#include <CAN.h>
#include <customCAN.h>


bool CANsend(_can_frame *sendPacket, void (*prepare)(_can_frame*)) {
  prepare(sendPacket);
  CAN.beginExtendedPacket(sendPacket->can_id,sendPacket->can_dlc);
  CAN.write(sendPacket->data,sendPacket->can_dlc);
  CAN.endPacket();
  return 1;
}

bool CANsend(_can_frame *sendPacket) {
  CAN.beginExtendedPacket(sendPacket->can_id,sendPacket->can_dlc);
  CAN.write(sendPacket->data,sendPacket->can_dlc);
  CAN.endPacket();
  return 1;
}

bool CANreceive(_can_frame *receivePacket, void (*interpret)(_can_frame*)) {
    uint16_t packetSize = CAN.parsePacket();
    byte i = 0; 

    receivePacket->can_id = CAN.packetId();
    receivePacket->can_dlc = packetSize;
    
    while (CAN.available()){
      receivePacket->data[i] = CAN.read(); i++;
    } Serial.println();

    // Call back
    interpret(receivePacket);
    Serial.println();

  return 1;
}

bool CANreceive(_can_frame *receivePacket) {
  // Read Message (Turn this into function that I must passed _can_frame reference)
  
    uint16_t packetSize = CAN.parsePacket();
    byte i = 0; 
    receivePacket->can_id = CAN.packetId();
    receivePacket->can_dlc = packetSize;
    
    while (CAN.available()){
      receivePacket->data[i] = CAN.read(); i++;
    } Serial.println();

  return 1;
}


/*------------------------------ CAN comminication Functions ----
-----(Check the spreadsheet in README.md for CAN ID custom rules)*/
// 1 CE 1 0A 00
// 0001 1100 1110 0001 0000 1010 0000 0000

// Create Extended CAN ID of Priority , BaseID , msg Number , src , dest 
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t BASE_ID, uint8_t MSG_NUM ,uint8_t SRC_ADDRESS, uint8_t DEST_ADDRESS) {
    uint32_t canID = 0;
    canID |= ((PRIORITY & 0xFF) << 24);        // (PP)Priority (bits 29)
    canID |= ((BASE_ID & 0x0F) << 20);         // (B) Base id denotes CAN channel (bit 28-20)
    canID |= ((MSG_NUM & 0xFF) << 16);         // (X) Message number (bits 19-16)
    canID |= ((SRC_ADDRESS & 0xFF) << 8);      // (SS)Source BMU address (bits 15-8)
    canID |= DEST_ADDRESS;                     // (DD)Destination BCU address (bits 7-0)
    return canID;
}

// Decode extended CAN ID
void decodeExtendedCANID(struct CANIDDecoded* myCAN ,uint32_t canID) {
    
    myCAN->PRIORITY = (canID >> 24) & 0xFF;        // Extract priority (bits 29)
    myCAN->BASE_ID = (canID >> 20) & 0x0F;         // Extract Message Number (bits 28-20)
    myCAN->MSG_NUM = (canID >> 16) & 0x0F;         // Extract Message Number (bits 19-16)
    myCAN->SRC = (canID >> 8) & 0xFF;              // Extract Source Address (bits 15-8)
    myCAN->DEST = canID & 0xFF;                    // Extract destination address (bits 7-0)
    
}

// BMS data structure