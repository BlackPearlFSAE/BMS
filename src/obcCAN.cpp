#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <util.h>

can_frame obcsent;
can_frame bmsreceived;

MCP2515 mcp2515(10,MCP_16MHZ);


void setup() {
  obcsent.can_id  = 0x18FF50E5 | CAN_EFF_FLAG;
  obcsent.can_dlc = 8;
  obcsent.data[0] = 0x03;
  obcsent.data[1] = 0x20; // 800 * 0.1V/s = 80 V Let's Report Back only 80 Volt charging
  obcsent.data[2] = 0x00;
  obcsent.data[3] = 0x32; // 5A fake data
  // obcsent.data[4] = 0b00010; // Send this status bit to be read from LSB order
  obcsent.data[4] = 0b00000; // Send this status bit to be read from LSB order
  obcsent.data[5] = 0x00; obcsent.data[6] = 0x00; obcsent.data[7] = 0x00; // doesn't matter
  Serial.begin(115200);
  
  mcp2515.reset();
  // mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- OBC to BMS Master ----------");
  // Serial.println("ID  DLC   DATA");
}

unsigned long last_time = 0;
void loop() {

  // Detect CAN message from on board Charger ONLY -- Other CAN
  // Polling the Message From on board charger (Every 500 ms)
  // This one priority should be listen 1st then send
  if(millis()-last_time >= 500) {

    if (mcp2515.readMessage(&bmsreceived) == MCP2515::ERROR_OK) {
      uint32_t parsedEXTId = bmsreceived.can_id & ~CAN_EFF_FLAG;
      Serial.print("ID: "); Serial.println(parsedEXTId, HEX); 
      Serial.print("DLC: ");Serial.println(bmsreceived.can_dlc, HEX);
      Serial.print("Data(Bytes): ");
      for(short i = 0; i < bmsreceived.can_dlc; i++) {
        Serial.print(bmsreceived.data[i],HEX); Serial.print(" ");
      } Serial.println();  

      if(parsedEXTId == 0x1806E5F4){
        
        uint8_t V_limitH = bmsreceived.data[0];
        uint8_t V_limitL = bmsreceived.data[1];
        uint8_t A_limitH = bmsreceived.data[2];
        uint8_t A_limitL = bmsreceived.data[3];
        float allowableVolt = mergeHLbyte(V_limitH,V_limitL)*0.1;
        float allowableAmp = mergeHLbyte(A_limitH,A_limitL)*0.1;
        // OBCVolt = mergeHLbyte(VoutH,VoutL);
        // OBCAmp = mergeHLbyte(AoutH,AoutL);
        Serial.print("Maximum Allowable Voltage by BMS: "); Serial.print(allowableVolt); Serial.println("V");
        Serial.print("Maximum Allowable Current by BMS: "); Serial.print(allowableAmp); Serial.println("A");

        switch (bmsreceived.data[4]) {
        case 0:
          Serial.println("Charger Start Operate");
          obcsent.data[0] = 0x03;
          obcsent.data[1] = 0x20; // 800 * 0.1V/s = 80 V fake data
          obcsent.data[2] = 0x00;
          obcsent.data[3] = 0x32; // 5A fake data
          // obcsent.data[4] = 0b00000; // in real scenario status bit will change according to real error
          break;
        
        case 1:
          /* Stop operate*/
          Serial.println("Charger Stop Operate");
          obcsent.data[0] = 0x00;
          obcsent.data[1] = 0x00; // 800 * 0.1V/s = 80 V fake data
          obcsent.data[2] = 0x00;
          obcsent.data[3] = 0x00; // 5A fake data
          // obcsent.data[4] = 0b00010; // Let's Assume Display Overheat STAT
          break;
        }
      }  
    }
    // Condition to see if charger stop charging 
    // Read the control bit , display whether BMS command is to charge or stop the charge
    // Display Max V and A from it
    
    mcp2515.sendMessage(MCP2515::TXB1,&obcsent); // Send Extended ID msg
    // Serial.println("Messages sent");
    last_time = millis();
  }
  
}
