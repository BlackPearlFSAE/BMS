//  MUST CHANGE CAN LIB to fully support Extended Frame Transmission & Reception like 
//

/* BMS Master unit Send And Receive to and from OBC*/

#include <Arduino.h>
#include <ArduinoSTL.h> // C/C++ standard libs comeback
#include <SPI.h>
#include <mcp2515.h>

// Byte conversion , bit shifting custom functions
#include <util.h>

can_frame bmssent;
can_frame obcreceived;
MCP2515 mcp2515(10,MCP_16MHZ);
#define SDCPIN 13 // actual test will use pin4

void setup() {
  
  digitalWrite(SDCPIN,HIGH);
  Serial.begin(115200);
  /* Set up BMS CAN frame*/
    bmssent.can_id  = 0x1806E5F4 | CAN_EFF_FLAG;
    bmssent.can_dlc = 8;
    // Conserved Byte
    bmssent.data[5] = 0x00;
    bmssent.data[6] = 0x00;
    bmssent.data[7] = 0x00;
  
  /* MCP2515 setup */
  mcp2515.reset(); // Reset SPI 
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- BMS ----------");
  // Serial.println("ID  DLC   DATA");
}

unsigned long last_time = 0;
void loop() {
  
  // /* BMS Native Feature*/
    //   float cellvolt = 5;
    //   float dischgAmp = 5;
    //   // Function to read individual Cell voltage once
    //   Serial.println();
    //   Serial.print("Cell Voltage: "); Serial.println(cellvolt,DEC);



  /* Minimum Voltage of 2 Module : 3.1*20 = 62V , Nominal Volage 3.6*20 = 72V
  Maximum allowable Voltage for 2 module : 83V => 830 => 0x 03 3E
  Maximum allowable current for 2 module : 5A => 50 => 0x 00 32 */
  // Polling the Message From on board charger (Every 500 ms)
  status STAT;

  // Transmitting Control Message (Every 500ms as well)
  if(millis()-last_time >= 500) {
    
    // This can be problematic as there needs to be a first message to make the OBC not entering COMMUNICATION ERROR
    // make sure bms both send first then recieve within 500ms. 
    // Condition 1 Normal BMS message during charge
    if(STAT.shutdownsig == 1) {
      bmssent.data[0] = 0x03; // V highbyte 
      bmssent.data[1] = 0x3E; // V lowbyte 83.0 V fake data
      bmssent.data[2] = 0x00; // A Highbyte
      bmssent.data[3] = 0x32; // A Lowbyte 5.0 A fake data
      bmssent.data[4] = 0x00; // Control Byte 0 charger operate
      // Not sure if there should be any change to V,A cap limit , just fixed the number by now
      // Send TTL LOW to Charging shutdown (Passed through builtin Relay driver there, fine.)
    } else {
      // Condition 2 Message During Shutdown
      bmssent.data[0] = 0x00; // V highbyte 
      bmssent.data[1] = 0x00; // V lowbyte
      bmssent.data[2] = 0x00; // A Highbyte
      bmssent.data[3] = 0x00; // A Lowbyte
      bmssent.data[4] = 0x01; // Control Byte 1 charger shutdown
      // Turn maximum allowable voltage and current to 0 also as fail safe
    } mcp2515.sendMessage(MCP2515::TXB1,&bmssent);
  
    if (mcp2515.readMessage(&obcreceived) == MCP2515::ERROR_OK) {
      uint32_t parsedEXTId = obcreceived.can_id & ~CAN_EFF_FLAG;
      Serial.print("ID: "); Serial.println(parsedEXTId, HEX); 
      Serial.print("DLC: ");Serial.println(obcreceived.can_dlc, HEX);
      Serial.print("Data(Bytes): ");
      for(int i = 0; i < obcreceived.can_dlc; i++) {
        Serial.print(obcreceived.data[i],HEX); Serial.print(" ");
      } Serial.println();

      // This block of code execute if detect CAN message from OBC ONLY--
      if(parsedEXTId== 0x18FF50E5) {
        // Monitor & Translate current Frame data
        uint8_t VoutH = obcreceived.data[0];
        uint8_t VoutL = obcreceived.data[1];
        uint8_t AoutH = obcreceived.data[2];
        uint8_t AoutL = obcreceived.data[3];
        float OBCVolt = mergeHLbyte(VoutH,VoutL)*0.1;
        float OBCAmp = mergeHLbyte(AoutH,AoutL)*0.1;
        // OBCVolt = mergeHLbyte(VoutH,VoutL);
        // OBCAmp = mergeHLbyte(AoutH,AoutL);
        Serial.print("Voltage from OBC: "); Serial.print(OBCVolt); Serial.println("V");
        Serial.print("Current from OBC: "); Serial.print(OBCAmp); Serial.println("A");
        
        /* Interpret OBC status, and decide on Shutdown command */
        uint8_t stat =  obcreceived.data[4]; // Status Byte
        checkstatLSB(&STAT,stat);

        // Shutdown if found any status bit as 1 (abnormal scenario)
        if(STAT.shutdownsig){
          digitalWrite(SDCPIN,HIGH);
          Serial.print("SHUTDOWN: OK ");
        } else {
          digitalWrite(SDCPIN,LOW);
          Serial.print("SHUTDOWN: NOT_OK ");
          // Shutdown , send the command to OBC to stop , then stop CAN
        }

        // Intepret Individual bit meaning
        Serial.print("OBC status: "); 
        for (short i =0 ; i <8 ; i++)
          Serial.print(STAT.statbin[i],DEC);
        Serial.println();
        
        switch (STAT.statbin[0]) {
          case 0:
            Serial.println("ChargerHW = Normal");
            break;
          
          case 1:
            Serial.println("ChargerHW = Faulty");
            break;
        }
        switch (STAT.statbin[1]) {
          case 0:
            Serial.println("ChargerTemp = Normal");
            break;
          
          case 1:
            Serial.println("ChargerTemp = Overheat");
            break;
        }
        switch (STAT.statbin[2]) {
          case 0:
            Serial.println("ChargerACplug = Normal");
            break;
          
          case 1:
            Serial.println("ChargerACplug = Reversed");
            break;
        }
        switch (STAT.statbin[3]) {
          case 0:
            Serial.println("Charger detects: Vbatt");
            break;
          
          case 1:
            Serial.println("Charger detects: ZERO Vbatt");
            break;
        }
        switch (STAT.statbin[4]) {
          case 0:
            Serial.println("COMMUNICATION STATUS: Normal");
            break;
          
          case 1:
            Serial.println("COMMUNICATION STATUS: Time out (6s)");
            break;
        }

          // How to read status byte
          // Detect Communication Error
          // communication timeout on the side of Charger (No command message from BMS is already dealt within OBC control system)
          // communication timeout of 6s on the side of BMS (6s of not receiving the status update from)
          // Charging Shutdown Emer button Should Cutout only HV or HV + Signal + LV power , if the

          // uint8_t swverH = canMsgRec.data[5];
          // uint8_t swverL = canMsgRec.data[6];
          // uint8_t hwver = canMsgRec.data[7];
      }
    }
    last_time = millis();
  }
}
