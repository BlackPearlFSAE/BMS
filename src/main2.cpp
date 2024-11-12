/* BCU
Minimum Voltage of 2 Module : 3.1*20 = 62V , Nominal Volage 3.6*20 = 72V
Maximum allowable Voltage for 2 module : 83V => 830 => 0x 03 3E
Maximum allowable current for 2 module : 5A => 50 => 0x 00 32
Priority Table, Check Charge -> NO
Read From SDC
Read from BMU
Read from BAMOCAR
RTOS and Push ROS topics
*/

/************************* Includes ***************************/
extern "C" {
  #include <driver/twai.h>
}
#include <Arduino.h>
#include <CAN.h>
#include <EEPROM.h>
// #include <freertos/

// utility function
#include <util.h>

/************************* Define macros *****************************/
#define SDCPIN 2 // actual test will use pin ...
#define OBCPIN 3 // This pin check for Signal from Charging Shutdown Circuit, indicates that it is charged

ESP32SJA1000Class CAN;
EEPROMClass EEPROM;

/**************** Local Function Delcaration *******************/
void bmsSendobc(_can_frame* bmssent);
void bmsReadobc(_can_frame* obcrecieve);
void CANwrite(bool rtr = 0);
void CANread();


/**************** Setup Variables *******************/

_can_frame sendmsg;
_can_frame receivemsg;
SDCstatus SDCstat;
unsigned long last_time = 0;
unsigned long beforeTimeout = 0;
bool TIMEOUT_FLAG = false; // Which condition will reset this flag? , Repress the emergency Button
int chargercond = true; // (May use External Interrupt later)

/*******************************************************************
  Setup Program
  Routine Program
********************************************************************/

void setup() {
  
  /* Shutdown System setup */
  pinMode(OBCPIN,INPUT_PULLDOWN);
  pinMode(SDCPIN,OUTPUT);
  digitalWrite(SDCPIN,HIGH); // BCU shutdown pin , LOW = Shutdown
  
  
  /* Communication Setup */
  Serial.begin(115200);
  CAN.setPins(DEFAULT_CAN_RX_PIN,DEFAULT_CAN_TX_PIN);
  if (!CAN.begin(STANDARD_BITRATE)) {
    Serial.println("Starting CAN failed!");
    while(1);
  }
  Serial.println("__BMS Master Initialized__");
}


void loop(){
  
  if(TIMEOUT_FLAG){
    SDCstat.shutdownsig = 0;
    Serial.println("BCU Detect COMMUNICATION Timeout");
  }
  if(SDCstat.shutdownsig == 1){
    digitalWrite(SDCPIN,HIGH);
  } else {
    digitalWrite(SDCPIN,LOW); Serial.println("!SHUTDOWN!");
  }

  /*---------------------------------------------Charging Event Routine*/
  // Condition to Check if the Charger is plugged 
  if(chargercond){

    // BCU OBC Communication (500ms cycle time)
    if(millis()-last_time >= 500) {
      
      // Set BCU canframe before sending to OBC according to shutdown state
      bmsSendobc(&sendmsg);
      CANwrite(); 

      // Read and interpret OBC canframe before deciding shutdown state
      CANread();
      bmsReadobc(&receivemsg);

      last_time = millis();
    }
  }

  /*---------------------------------------------Driving Event Routine*/

  // BCU CMD <-> BMU Module Report
  if(millis()-last_time >= 100){
    //
    //CANwrite();

    // CANread();
    //
  }

  //  <- BMU Cell monitoring
  if(millis()-last_time >= 200){
    //
    //CANwrite();

    // CANread();
    //
  }


}

/*******************************************************************
  Local Functions Definition
********************************************************************/

void CANwrite(bool rtr = 0){
  CAN.beginExtendedPacket(sendmsg.can_id, sendmsg.can_dlc,rtr);
  CAN.write(sendmsg.data,sendmsg.can_dlc);
  CAN.endPacket();
}

void CANread(){
  // Read Message (Turn this into function that I must passed _can_frame reference)
    uint16_t packetSize = CAN.parsePacket();
    byte i = 0; 
    if (packetSize || CAN.packetId() != -1) {
      receivemsg.can_id = CAN.packetId();
      receivemsg.can_dlc = packetSize;

      Serial.print("ID:"); Serial.println(CAN.packetId(), HEX);
      Serial.print("DLC: "); Serial.println(packetSize);
      Serial.print("Data(Bytes): ");
      
      while (CAN.available()){
        receivemsg.data[i] = CAN.read(); i++;
        Serial.print(receivemsg.data[i],HEX); Serial.print(" ");
      } Serial.println();
      // Update Communication timer
      beforeTimeout = millis();
    }
    // Check Communication timeout
    else {
      if(millis()-beforeTimeout >= 6000)
        !TIMEOUT_FLAG; 
    }
}


void bmsSendobc(_can_frame* bmssent){
  // There needs to be a 1st message to make the OBC not entering COMMUNICATION ERROR
  
      /* Set up BMS CAN frame*/
    bmssent->can_id  = 0x1806E5F4;
    bmssent->can_dlc = 8;

    // Condition 1 Normal BMS message during charge
    if(SDCstat.shutdownsig == 1) {
      bmssent->data[0] = 0x02; // V highbyte 
      bmssent->data[1] = 0xD0; // V lowbyte 72.0 V fake data -> Range 69-72-74 V
      bmssent->data[2] = 0x00; // A Highbyte
      bmssent->data[3] = 0x32; // A Lowbyte 5.0 A fake data
      bmssent->data[4] = 0x00; // Control Byte 0 charger operate
    } else {
      // Condition 0 Shutdown message
      bmssent->data[0] = 0x00; // V highbyte 
      bmssent->data[1] = 0x00; // V lowbyte
      bmssent->data[2] = 0x00; // A Highbyte
      bmssent->data[3] = 0x00; // A Lowbyte
      bmssent->data[4] = 0x01; // Control Byte 1 charger shutdown
    } 

    // Initialize all Reserved Byte as 0x00
    bmssent->data[5] = 0x00;
    bmssent->data[6] = 0x00;
    bmssent->data[7] = 0x00;
}

void bmsReadobc(_can_frame* obcreceived){
  // This block of code execute if detect CAN message from OBC ONLY--
      if(CAN.packetId() == 0x18FF50E5) {
        // Monitor & Translate current Frame data
        uint8_t VoutH = obcreceived->data[0];
        uint8_t VoutL = obcreceived->data[1];
        uint8_t AoutH = obcreceived->data[2];
        uint8_t AoutL = obcreceived->data[3];
        float OBCVolt = mergeHLbyte(VoutH,VoutL)*0.1;
        float OBCAmp = mergeHLbyte(AoutH,AoutL)*0.1;
        // OBCVolt = mergeHLbyte(VoutH,VoutL);
        // OBCAmp = mergeHLbyte(AoutH,AoutL);
        Serial.print("Voltage from OBC: "); Serial.print(OBCVolt); Serial.println("V");
        Serial.print("Current from OBC: "); Serial.print(OBCAmp); Serial.println("A");
        
        /* Interpret OBC status, and decide on Shutdown command */
          uint8_t stat =  receivemsg.data[4]; // Status Byte
          checkstatLSB(&SDCstat,stat);

        // Intepret Individual bit meaning
        Serial.print("OBC status: "); 
        for (short i =0 ; i <8 ; i++)
          Serial.print(SDCstat.statbin[i],DEC);
        Serial.println();


        switch (SDCstat.statbin[0]) {
          case 1:
            Serial.println("ChargerHW = Faulty");
            break;
        }
        switch (SDCstat.statbin[1]) {
          case 1:
            Serial.println("ChargerTemp = Overheat");
            break;
        }
        switch (SDCstat.statbin[2]) {
          case 1:
            Serial.println("ChargerACplug = Reversed");
            break;
        }
        switch (SDCstat.statbin[3]) {
          case 1:
            Serial.println("Charger detects: ZERO Vbatt");
            break;
        }
        switch (SDCstat.statbin[4]) {
          case 1:
            Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
            break;
        } 
      } 
}


void bcuSendbmu() {

}
void bcuReadSDC(){

}