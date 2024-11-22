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
// #include <ArduinoSTL.h>
#include <CAN.h>
#include <EEPROM.h>
// #include <freertos/

// utility function
#include <util.h>

/************************* Define macros *****************************/
#define SDCIN 2   // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty SDC
#define OBCIN 3   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
#define BSPDIN 4  // Check BSPD OK signal from BSPD directly 
#define IMDIN 5   // Check IMD OK signal from IMD directly
#define BMSOUT 6  // OUTPUT Fault Signal to BMS relay
// ESP32SJA1000Class CAN;


/**************** Local Function Delcaration *******************/
void BCUtoOBCWrite(_can_frame* BCUsent);
void BCUreadOBC(_can_frame* BCUrecevid);
void BCUtoBMUwrite(_can_frame* BCUsent);
void BCUreadBMU(_can_frame* BCUreceived);
void BCUReadSDC();
void CANsend();
void CANreceive();


/**************** Setup Variables *******************/

const int initFlagAddress = 0;  // Address to store initialization flag
const byte initFlagValue = 0xAA; // Arbitrary non-zero value indicating initialized
// Global Struct
_can_frame sendmsg;
_can_frame receivemsg;
SDCstatus SDCstat;
bool BCUSHUTDOWN_STATE = 1;
unsigned long last_time = 0;
unsigned long last_time2 = 0;
unsigned long beforeTimeout = 0;
bool TIMEOUT_FLAG = false; // Which condition will reset this flag? , Repress the emergency Button
bool CHARGING_FLAG = false; // (May use External Interrupt to change this flag later)
bool eepromWriteFlag = false; // Change this value to true if you want to update Default Parameter

uint8_t *tempptr = nullptr; // General Purpose pointer , use for any temporary placeholder


//Class or Struct for Data logging ------------------------*****************------------------*//
/*--------------------------------------------------------------------------------------------*/

/*******************************************************************
  Setup Program
  Routine Program
********************************************************************/

void setup() {
  
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(BMSOUT,OUTPUT);
  // In case of specific troubleshooting
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  
  
  /* Write Default Parameter to EEPROM only once*/
  if (eepromWriteFlag) {
    Serial.println("EEPROM is being initialized.");
    EEPROM.write(0,4); // Timeout (byte) 4/1
    EEPROM.write(1,42); // VcellMax (byte) 4.2/0.1
    EEPROM.write(2,32); // VcellMin (byte) 3.2/0.1
    EEPROM.write(3,60); // TempMax (byte) 60 / 1
    EEPROM.write(4,2); // dvMax (byte) 0.2/0.1
    Serial.println("Initialization complete.");

    // Display EEPROM Data
    byte defaultParam;
    for(short i; i<5 ;i++){
      EEPROM.get(i,defaultParam);
      Serial.println(defaultParam);
    }

  } else {
    Serial.println("EEPROM already initialized.");
    // Display EEPROM Data
    byte defaultParam;
    for(short i; i<5 ;i++){
      EEPROM.get(i,defaultParam);
      Serial.println(defaultParam);
    }
  }

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
  
  // ! this  Condtion may reset shutdown status unknowingly

  // This section reads SDC , CHG_SDC , IMD, BSPD signal , proper logic shifting is needed

  (digitalRead(SDCIN) || digitalRead(OBCIN)) ? (SDCstat.SHUTDOWN_OK = 1) : (SDCstat.SHUTDOWN_OK = 0);
  (digitalRead(IMDIN)) ? (SDCstat.IMD_OK = 1) : (SDCstat.IMD_OK = 0);
  (digitalRead(BSPDIN)) ? (SDCstat.IMD_OK = 1) : (SDCstat.BSPD_OK = 0);
  
  
  // Any form of CAN communication timeout will Shutdown
  if(TIMEOUT_FLAG){
    SDCstat.SHUTDOWN_OK = 0;
    Serial.println("BCU Detect COMMUNICATION Timeout");
  }
  
  // Set BCUSHUTDOWN_STATE to 0, if fault detects. or SDCIN , OBCIN read 0
  if(SDCstat.SHUTDOWN_OK == 1){
    digitalWrite(BMSOUT,HIGH);
  } else {
    digitalWrite(BMSOUT,LOW); Serial.println("!SHUTDOWN!");
  }

  // Reconfirm if the Charger is actually plugged 
  (digitalRead(OBCIN)) ? (CHARGING_FLAG = true) : (CHARGING_FLAG = false);

  /*---------------------------------------------Charging Event Routine*/
  
  // BCU OBC Communication (500ms cycle time)
  if(millis()-last_time >= 500 && CHARGING_FLAG) {
    
    // Set BCU canframe before sending to OBC according to shutdown state
    BCUtoOBCWrite(&sendmsg);
    CANsend(); 
    // Read and interpret OBC canframe before deciding shutdown state
    CANreceive();
    BCUreadOBC(&receivemsg);

    last_time = millis();
  }

  /*---------------------------------------------Driving Event Routine*/

  // BCU CMD <-> BMU Module Report
  if(millis()-last_time >= 100){
    BCUtoBMUwrite(&sendmsg);
    CANsend(); // Broadcast to BMU
    CANreceive();
    BCUreadBMU(&sendmsg);
  }

  //  <- BMU Cell monitoring
  if(millis()-last_time >= 200){
    
    CANreceive();
    BCUreadBMU(&receivemsg);
    last_time2 = millis();
  }

}

/*******************************************************************
  Local Functions Definition
********************************************************************/

void CANsend(){
  CAN.beginExtendedPacket(sendmsg.can_id, sendmsg.can_dlc);
  CAN.write(sendmsg.data,sendmsg.can_dlc);
  CAN.endPacket();
}

void CANreceive(){
  // Read Message (Turn this into function that I must passed _can_frame reference)
    uint16_t packetSize = CAN.parsePacket();
    byte i = 0; 
    if (CAN.parsePacket() || CAN.packetId() != -1) {
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
    else if(millis()-beforeTimeout >= 6000) {
      !TIMEOUT_FLAG; 
    }
}


void BCUtoOBCWrite(_can_frame* BCUsent){
  // There needs to be a 1st message to make the OBC not entering COMMUNICATION ERROR
    /* Set up BMS CAN frame*/
    BCUsent->can_id  = 0x1806E5F4;
    BCUsent->can_dlc = 8;

    // Condition 1 Normal BMS message during charge
    if(SDCstat.SHUTDOWN_OK == 1) {
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


void BCUreadOBC(_can_frame* BCUreceived){
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
        uint8_t *statusbit_array = checkstatLSB(stat);
        // checkstatLSB(&SDCstat,stat);

      // Intepret Individual bit meaning
      Serial.print("OBC status: "); 
      for (short i =0 ; i <8 ; i++)
        Serial.print(statusbit_array[i],HEX);
      Serial.println();

      switch (statusbit_array[0]) {
        case 1:
          Serial.println("ChargerHW = Faulty");
          break;
      }
      switch (statusbit_array[1]) {
        case 1:
          Serial.println("ChargerTemp = Overheat");
          break;
      }
      switch (statusbit_array[2]) {
        case 1:
          Serial.println("ChargerACplug = Reversed");
          break;
      }
      switch (statusbit_array[3]) {
        case 1:
          Serial.println("Charger detects: ZERO Vbatt");
          break;
      }
      switch (statusbit_array[4]) {
        case 1:
          Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
          break;
      } 
    } 
}


void BCUtoBMUwrite(_can_frame* BCUsent){
    byte Timeout = EEPROM.read(0);
    byte VmaxCell = EEPROM.read(1);
    byte VminCell = EEPROM.read(2);
    byte Tmax = EEPROM.read(3);
    byte dVmax = EEPROM.read(4);

    /* Set up BMS CAN frame*/
    BCUsent->can_id  = 0x0CE00000;
    BCUsent->can_dlc = 8;

    // Condition 1 Charging Event
    // Condition 2 Driving Event
    if(SDCstat.SHUTDOWN_OK == 1) {
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


void BCUreadBMU(_can_frame* BCUreceived){
  // Function to concatenate and split string??


  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->can_id));

  float V_cell[10];
  float Temp_cell[3];
  int currentsense;
  float dv = 0.0;

  u_int8_t optstatusbit = 0b00000000;
  u_int16_t balanceNum = 0b00000000000;
  u_int16_t faultcellnum = 0b00000000000;
  
  // Layout all relevant data for ROS topics
  
  if(decodedCANID.PRIORITY == 0x01 && decodedCANID.BASE_ID == 0xCE && decodedCANID.DEST == 0xE5){
    switch (decodedCANID.MSG_NUM) {
    
    // Case msgnum = 1 , // Opt status , faultcode, faultcode cell number
    // There's still a problem if each of the cell somehow exhibit more than 1 fault? , would that mean the same message must be tx?
    case 1:
      bool &bcushutdown_state = BCUSHUTDOWN_STATE; // Alias name

      Serial.print("BMU Operation Status (LSB)"); Serial.print(":");
      optstatusbit = BCUreceived->data[0];
      uint8_t *OPT_BITARRAY = checkstatLSB(BCUreceived->data[0]);
      for (short i=0; i < 8; i++){
        Serial.print(OPT_BITARRAY[i]);
        if(OPT_BITARRAY[i] == 1)
          bcushutdown_state = false;
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
        Serial.println(V_cell[i]); Serial.print("V");
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

          Serial.print("Discharged Current"); Serial.print(":");
          currentsense = BCUreceived->data[i];
          Serial.print(currentsense); Serial.println("A");

        } else {
          Serial.print("Cell balancing discharge"); Serial.print(":");
          balanceNum = BCUreceived->data[i];
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

// Yuil sketch function to publish ROS topics