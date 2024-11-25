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
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
// #include <ArduinoSTL.h>
#include <CAN.h>
#include <EEPROM.h>
// #include <freertos/

// utility function
#include <util.h>

/************************* Define macros *****************************/
#define SDCIN 9   // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty SDC
#define OBCIN 10   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
#define BSPDIN 13  // Check BSPD OK signal from BSPD directly 
#define IMDIN 14   // Check IMD OK signal from IMD directly
#define BMSOUT LED_BUILTIN   // OUTPUT Fault Signal to BMS relay
#define EEPROM_SIZE 5


/**************** Local Function Delcaration *******************/
void BCUtoOBCWrite(_can_frame* BCUsent);
void BCUreadOBC(_can_frame* BCUrecevid);
void BCUtoBMUwrite(_can_frame* BCUsent);
void BCUreadBMU(_can_frame* BCUreceived);
void CANsend(_can_frame *sendPacket, void (*prepare)(_can_frame*));
void CANreceive(_can_frame *receivePacket, void (*interpret)(_can_frame*));


/**************** Setup Variables *******************/

const int initFlagAddress = 0;  // Address to store initialization flag
const byte initFlagValue = 0xAA; // Arbitrary non-zero value indicating initialized
// Global Struct
_can_frame sendmsg;
_can_frame receivemsg;
SDCstatus SDCstat;
bool BCUSHUTDOWN_STATE = 1;
unsigned long reference_time = 0;
unsigned long reference_time2 = 0;
unsigned long beforeTimeout = 0;
bool TIMEOUT_FLAG = false; // Which condition will reset this flag? , Repress the emergency Button
bool CHARGING_FLAG = false; // (May use External Interrupt to change this flag later)
bool eepromWriteFlag = false; // Change this value to true if you want to update Default Parameter

byte Timeout = EEPROM.read(0);
byte VmaxCell = EEPROM.read(1);
byte VminCell = EEPROM.read(2);
byte Tmax = EEPROM.read(3);
byte dVmax = EEPROM.read(4);


//Class or Struct for Data logging ------------------------*****************------------------*//
/*--------------------------------------------------------------------------------------------*/

/*******************************************************************
  Setup Program
  Routine Program
********************************************************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable detector
  Serial.begin(115200);
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(BMSOUT,OUTPUT);
  // // In case of specific troubleshooting
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  
  
  /* Write Default Parameter to EEPROM only once*/
  EEPROM.begin(EEPROM_SIZE); // only for esp32 , that use virtual eeprom
  if (eepromWriteFlag) {
    Serial.println("write to EEPROM.");
    EEPROM.write(0,4); // Timeout (byte) 4/1
    EEPROM.write(1,42); // VcellMax (byte) 4.2/0.1
    EEPROM.write(2,32); // VcellMin (byte) 3.2/0.1
    EEPROM.write(3,60); // TempMax (byte) 60 / 1
    EEPROM.write(4,2); // dvMax (byte) 0.2/0.1
    EEPROM.commit();
    Serial.println("Writing to EEPROM. complete");
    
    // Display EEPROM Data
    Serial.println("EEPROM Data:");
    byte defaultParam;
    for(short i; i<EEPROM_SIZE ;i++){
      EEPROM.get(i,defaultParam);
      Serial.println(defaultParam);
    }

  } else {
    // Display EEPROM Data
    Serial.println("EEPROM Data:");
    byte defaultParam;
    for(short i; i<EEPROM_SIZE ;i++){
      EEPROM.get(i,defaultParam);
      Serial.println(defaultParam);
    }
  }

  /* Communication Setup */
  CAN.setPins(DEFAULT_CAN_RX_PIN,DEFAULT_CAN_TX_PIN);
  // CAN.filterExtended(id);
  // CAN.filterExtended(id, mask);
  if (!CAN.begin(STANDARD_BITRATE)) {
    Serial.println("Starting CAN failed!");
    while(1);
  }
  Serial.println("__BMS Master Initialized__");
  
  // Dummy Setting , set what I want to be true
    CHARGING_FLAG = false;
    SDCstat.BMS_OK =1;
    SDCstat.IMD_OK = 1;
    SDCstat.BSPD_OK = 1;
  
}

bool timeoutprev;
void loop(){

  // This section reads SDC , CHG_SDC , IMD, BSPD signal , proper logic shifting is needed
  // Read SHUTDOWN_OK_SIGNAL signal according to the actual physical voltages of shutdown circuit output
  (digitalRead(SDCIN) || digitalRead(OBCIN)) ? (SDCstat.SHUTDOWN_OK_SIGNAL = 1) : (SDCstat.SHUTDOWN_OK_SIGNAL = 0);
  // Read IMD_Ok and BSPD_OK status from shutdown circuit
  (digitalRead(IMDIN)) ? (SDCstat.IMD_OK = 1) : (SDCstat.IMD_OK = 0);
  (digitalRead(BSPDIN)) ? (SDCstat.IMD_OK = 1) : (SDCstat.BSPD_OK = 0);
  
  // Confirm if the Charger is actually plugged 
  (digitalRead(OBCIN)) ? (CHARGING_FLAG = true) : (CHARGING_FLAG = false);

  // BMS_OK flag is determined in this mcu
  
  // Any form of Communication Timeout triggers BMS fault => Shutdown
  if(TIMEOUT_FLAG == true){
    // Keep previous state of timeout
    // timeoutprev = TIMEOUT_FLAG;
    SDCstat.BMS_OK = 0;
    if(millis()-reference_time >= 300)
      Serial.println("BCU detected Communication Timeout");
  }
   
  // // If previous state of TIMEOUT_FLAG is true , reset shutdown signal
  // else if(!TIMEOUT_FLAG == true)
  //   SDCstat.BMS_OK = 1;
  // // if previous sate of TIMEOUT_FLAG is false , it goes back to Shutdown again ,
  // after the third condition , next loop will run the first condition


  // case 1 , Both operate Normally , BMS doesn't detect fault , and Shutdown signal is still high
  if(SDCstat.BMS_OK == 1 && SDCstat.SHUTDOWN_OK_SIGNAL == 1){
    digitalWrite(BMSOUT,HIGH);
    // Reaffirm
    SDCstat.BMS_OK = 1;
    SDCstat.SHUTDOWN_OK_SIGNAL = 1;
  } 
  
  // case 3 BMS doesn't detect fault, but Shutdown signal already LOW due to other system fault : HIGH & LOW
  else if (SDCstat.BMS_OK == 1 || SDCstat.SHUTDOWN_OK_SIGNAL == 0){
      
  }

  // case 2 BMS detect fault, while Shutdown signal is currently HIGH , confirmed fault caused by BMS : LOW & HIGH
  else if (SDCstat.BMS_OK == 0 && SDCstat.SHUTDOWN_OK_SIGNAL == 1){
    digitalWrite(BMSOUT,LOW); 
   // Reset CAN frame stucture
    sendmsg = _can_frame();
    receivemsg = _can_frame();

    if(millis()-reference_time >= 300)
      Serial.println("!SHUTDOWN!");
  }

  // case 4 , Both at fault 
  else {
    digitalWrite(BMSOUT,LOW); 
   // Reset CAN frame stucture
    sendmsg = _can_frame();
    receivemsg = _can_frame();

    if(millis()-reference_time >= 300)
      Serial.println("!SHUTDOWN!");
  }

  /*---------------------------------------------Driving Event Routine*/

  // BCU CMD <-> BMU Module Report
  if(millis()-reference_time >= 200){

    // CANsend(&sendmsg,BCUtoBMUwrite); // Broadcast to BMU
    // CANreceive(&receivemsg,BCUreadBMU);


    // The problem occurs when CAN msg no longer recieved , but interpretation still keep the message going even after shutdown
    // At shutdown we can do 2 things
    // 1. reset can frame structure, but it is temporary , 
    // 2. make CAN receive , send , recieve call back argument, by passing function name -> bcu write, bcu...
  }

  //  <- BMU Cell monitoring
  if(millis()-reference_time >= 300){

    CANreceive(&receivemsg,BCUreadBMU);
    reference_time = millis();
  }

/*---------------------------------------------Charging Event Routine*/
  
  // BCU OBC Communication (500ms cycle time)
  if(millis()-reference_time2 >= 500 && CHARGING_FLAG) {
    
    // Prepare CAN frame , then sent
    // Receive CAn frame , then interpret
    CANsend(&sendmsg,BCUtoOBCWrite);
    CANreceive(&receivemsg,BCUreadOBC);


    reference_time2 = millis();
  }
} // Publish as ROS topics
 
/*******************************************************************
  Local Functions Definition
********************************************************************/

// Why message transmit so much , it's like transmitting , 8 packet per cycle time?
void CANsend(_can_frame *sendPacket, void (*prepare)(_can_frame*)) {
  prepare(sendPacket);
  CAN.beginExtendedPacket(sendPacket->can_id, sendPacket->can_dlc);
  // CAN.write(sendPacket->data[0]);
  // CAN.write(sendPacket->data[1]);
  // CAN.write(sendPacket->data[2]);
  // CAN.write(sendPacket->data[3]);
  // CAN.write(sendPacket->data[4]);
  // CAN.write(sendPacket->data[5]);
  // CAN.write(sendPacket->data[6]);
  // CAN.write(sendPacket->data[7]);
  CAN.write(sendPacket->data,sendPacket->can_dlc);
  CAN.endPacket();
}
// use function pointer to pass a call back
void CANreceive(_can_frame *receivePacket, void (*interpret)(_can_frame*)) {
  // Read Message (Turn this into function that I must passed _can_frame reference)
    uint16_t packetSize = CAN.parsePacket();
    byte i = 0; 

    if (CAN.parsePacket() || CAN.peek() != -1) {
      // if (CAN.parsePacket() || CAN.packetId() != -1) {
      receivePacket->can_id = CAN.packetId();
      receivePacket->can_dlc = packetSize;

      Serial.print("ID:"); Serial.println(CAN.packetId(), HEX);
      Serial.print("DLC: "); Serial.println(packetSize);
      Serial.print("Data: ");
      
      while (CAN.available()){
        receivePacket->data[i] = CAN.read(); i++;
        Serial.print(receivePacket->data[i],HEX); Serial.print(" ");
      } Serial.println();

      // Call back
      interpret(receivePacket);

      // Update Communication timer
      TIMEOUT_FLAG = false;
      beforeTimeout = millis();
    } 
    // If receiving nothingCheck Communication timeout
    else if (millis()-beforeTimeout >= Timeout*1000){
      TIMEOUT_FLAG = true;
    }
    
}


void BCUtoOBCWrite ( _can_frame* BCUsent ) {
  // There needs to be a 1st message to make the OBC not entering COMMUNICATION ERROR
    /* Set up BMS CAN frame*/
    BCUsent->can_id  = 0x1806E5F4;
    BCUsent->can_dlc = 8;

    // Condition 1 Normal BMS message during charge
    if(SDCstat.BMS_OK == 1) {
      BCUsent->data[0] = 0x02; // V highbyte 
      BCUsent->data[1] = 0xD0; // V lowbyte 72.0 V fake data -> Range 69-72-74 V
      BCUsent->data[2] = 0x00; // A Highbyte
      BCUsent->data[3] = 0x32; // A Lowbyte 5.0 A fake data
      BCUsent->data[4] = 0x00; // Control Byte 0 charger operate
    } else {
      // Condition 0 Shutdown message
      BCUsent->data[0] = 0x00; // V highbyte 
      BCUsent->data[1] = 0x00; // V lowbyte
      BCUsent->data[2] = 0x00; // A Highbyte
      BCUsent->data[3] = 0x00; // A Lowbyte
      BCUsent->data[4] = 0x01; // Control Byte 1 charger shutdown
    } 

    // Initialize all Reserved Byte as 0x00
    BCUsent->data[5] = 0x00;
    BCUsent->data[6] = 0x00;
    BCUsent->data[7] = 0x00;
}


void BCUreadOBC ( _can_frame* BCUreceived ) {
    if(BCUreceived->can_id == 0x18FF50E5) {
      // Monitor & Translate current Frame data
      uint8_t VoutH = BCUreceived->data[0];
      uint8_t VoutL = BCUreceived->data[1];
      uint8_t AoutH = BCUreceived->data[2];
      uint8_t AoutL = BCUreceived->data[3];
      float OBCVolt = mergeHLbyte(VoutH,VoutL)*0.1;
      float OBCAmp = mergeHLbyte(AoutH,AoutL)*0.1;
      Serial.print("Voltage from OBC: "); Serial.print(OBCVolt); Serial.println("V");
      Serial.print("Current from OBC: "); Serial.print(OBCAmp); Serial.println("A");
      
      /* Interpret OBC status, and decide on Shutdown command */
        uint8_t stat =  BCUreceived->data[4]; // Status Byte
        uint16_t *bitarray_holder = checkstatLSB(stat);
        
        for (short i=0; i < 8; i++){
          // Finding 1 inside the bitarray indicates fault status
          
        } Serial.println();

      // Intepret Individual bit meaning
      Serial.print("OBC status: "); 
      for (short i =0 ; i <8 ; i++){
        Serial.print(bitarray_holder[i],HEX);
        if(bitarray_holder[i] == 1)
            SDCstat.BMS_OK = 0;
        }
      Serial.println();

      switch (bitarray_holder[0]) {
        case 1:
          Serial.println("ChargerHW = Faulty");
          break;
      }
      switch (bitarray_holder[1]) {
        case 1:
          Serial.println("ChargerTemp = Overheat");
          break;
      }
      switch (bitarray_holder[2]) {
        case 1:
          Serial.println("ChargerACplug = Reversed");
          break;
      }
      switch (bitarray_holder[3]) {
        case 1:
          Serial.println("Charger detects: ZERO Vbatt");
          break;
      }
      switch (bitarray_holder[4]) {
        case 1:
          Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
          break;
      } 
    } 
}

void BCUtoBMUwrite ( _can_frame* BCUsent ) {
    
    /* Set up BMS CAN frame*/
    BCUsent->can_id  = 0x0CE00000;
    BCUsent->can_dlc = 8;

    // Condition 1 Charging Event
    // Condition 2 Driving Event
    if(SDCstat.BMS_OK == 1) {
      BCUsent->data[0] = 0b0000; 
    } else {
      BCUsent->data[0] = 0x00; // V highbyte 
    } 
    BCUsent->data[2] = (byte)Timeout/1;
    BCUsent->data[3] = (byte)VmaxCell/0.1; 
    BCUsent->data[4] = (byte)VminCell/0.1; 
    BCUsent->data[5] = (byte)Tmax/1; 
    BCUsent->data[6] = (byte)dVmax/0.1;
    // Resereved byte
    BCUsent->data[7] = 0x00;
}


void BCUreadBMU ( _can_frame* BCUreceived ) {
  // Function to concatenate and split string??
  uint16_t *bitarray_holder = nullptr; // General Purpose pointer , use for any temporary placeholder

  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->can_id));

  float V_cell[10];
  float Temp_cell[3];
  int currentsense;
  float dv = 0.0;

  uint8_t optstatusbit = 0b00000000; // 8 bit status
  uint16_t balanceNum = 0b00000000000; // 10 bit status
  uint16_t faultcellnum = 0b00000000000;
  
  // Layout all relevant data for ROS topics
  // Priority 1 , Base ID : CE or BMS , Destination E5 or BCU
  if(decodedCANID.PRIORITY == 0x01 && decodedCANID.BASE_ID == 0xCE && decodedCANID.DEST == 0xE5){
    switch (decodedCANID.MSG_NUM) {
    
    // Case msgnum = 1 , // Opt status , faultcode, faultcode cell number
    // There's still a problem if each of the cell somehow exhibit more than 1 fault? , would that mean the same message must be sent?
    case 1:
      
      Serial.print("BMU Operation Status (LSB)"); Serial.print(":");
      optstatusbit = BCUreceived->data[0];
      bitarray_holder = checkstatLSB(optstatusbit);
      for (short i=0; i < 8; i++){
        Serial.print(bitarray_holder[i]);
        // Finding 1 inside the bitarray indicates fault status
        if(bitarray_holder[i] == 1)
          SDCstat.BMS_OK = 0;
      } Serial.println();
      // Interpret BitArray
        // Save Data to Data Log data
        // Datalog = OPT_BITARRAY

      // Change this one to instead of passing specific struct, just return bool , and use external if-else to set the struct stat
      // also return stat array (dynamic array) , then we can shove that statarray to any struct array later 
      // SDC status will now be enumerator? with 3 levels of 
      Serial.println();
      break;

    // Case msgnum = 2 , 8 cell voltages
    case 2:
      
      Serial.print("Module No.");Serial.println(decodedCANID.SRC);
      for(short i; i<8; i++){
        Serial.print("VCell"); Serial.print(i+1); Serial.print(":");
        V_cell[i] = BCUreceived->data[i]*0.02;
        Serial.print(V_cell[i]); Serial.println("V");
      }
      break;
    
    // Case msgnum =  3, 2 cell voltages , 3 cell temperature, 1 (Current sense), and cell balancing discharge bit
    case 3:
      Serial.print("Module No.");Serial.println(decodedCANID.SRC);
      for(short i; i<8; i++){
        if( i<2 ) {
          Serial.print("VCell"); Serial.print(9+i); Serial.print(":");
          V_cell[i] = BCUreceived->data[i]*0.02;
          Serial.print(V_cell[i]); Serial.println("V");
        } else if( i<5 ) {

          Serial.print("Temp"); Serial.print(i+1); Serial.print(":");
          Temp_cell[i-2] = BCUreceived->data[i]*0.0125;
          Serial.print(Temp_cell[i-2]); Serial.println("C");
          
        } else if ( i == 5 ) {
          // May change to direct sensor reading 
          Serial.print("Module Discharged Current"); Serial.print(":");
          currentsense = BCUreceived->data[i];
          Serial.print(currentsense); Serial.println("A");

        } else {
          Serial.print("Cell balancing discharge"); Serial.print(":");
          balanceNum = BCUreceived->data[i];
          bitarray_holder = checkstatLSB(balanceNum);
          for (short i=0; i < 8; i++){
            Serial.print(bitarray_holder[i]);
            // Finding 1 inside the bitarray indicates fault status
            if(bitarray_holder[i] == 1)
              SDCstat.BMS_OK = 0;
          } Serial.println();

          // Interpret bit array to see which cell is performing cell balancing
          // Need function to check binary position in LSB or MSB,
          // checkstatLSB(); 
          // Change this one to instead of passing specific struct, just return bool , and use external if-else to set the struct stat
          // also return stat array (dynamic array) , then we can shove that statarray to any struct array later 
          Serial.println(balanceNum); 
        }
      }
      break;
    }
    
  }


}

// BMS read SDC function

// Yuil sketch function to publish ROS topics