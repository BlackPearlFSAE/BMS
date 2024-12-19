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
// extern "C" {
//   #include <driver/twai.h>
// }
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
// #include <micro_ros_arduino.h>
// #include <freertos/
// utility function

#include <Arduino.h>
#include <CAN.h>
#include <EEPROM.h>
// #include <arduino-timer.h>
#include <customCAN.h>
#include <util.h>
// #include <vector>

/************************* Define macros *****************************/
#define SDCIN 9   // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty SDC
#define OBCIN 10   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
#define BSPDIN 13  // Check BSPD OK signal from BSPD directly 
#define IMDIN 14   // Check IMD OK signal from IMD directly
#define BMSOUT LED_BUILTIN   // OUTPUT Fault Signal to BMS relay
#define EEPROM_SIZE 5
#define OBC_SYNC_TIME 500
#define SYNC_TIME 200
#define TIMEOUT_TIME 4000

#define VMAX_CELL 4.2
#define VMIN_CELL 3.2
#define TEMP_MAX_CELL 60
#define DVMAX 0.2
#define CELL_NUM 16
#define TEMP_SENSOR_NUM 2
#define BMU_NUM 8
/**************** Local Function Delcaration *******************/

void BCUtoOBC_cmd_messagePrep ( _can_frame* BCUsent , bool cmd);
void BCUtoBMU_cmd_messagePrep ( _can_frame* BCUsent);
void BCUmonitorOBC ( _can_frame* BCUreceived );
void debugBMUmsg();
void debugBMUFault();
void debugOBCmsg();
void BCUmonitorBMU ( _can_frame* BCUreceived);

/**************** Setup Variables *******************/
// Maybe I'll change them to typedef in case of C compatibility??

// CAN structure
_can_frame sendmsg;
_can_frame receivemsg;
// _can_frame receivemsg[8];

// Data Aggregation , and Relay to Other Sub System , e.g. Telemetry , DataLogger, BMS GUI

// Physical condition of LV Circuit , safety information , and relay it to Telemetry system
struct SDCstatus {
  bool SHUTDOWN_OK_SIGNAL = 1; // AIR+
  bool BMS_OK = 1; // AMS_OUT+ ,AMS_GND
  bool IMD_OK = 1; // IMD_OUT
  bool BSPD_OK = 1; // BSPD_OUT
} SDCstat;

// Report BMS data, and Faulty status code // Serialize this data in 8 bit buffer
struct BMSdata {

  // Basic BMU Data
  uint8_t V_CELL[CELL_NUM];
  uint8_t TEMP_SENSE[TEMP_SENSOR_NUM];
  uint8_t V_MODULE = 0;
  uint8_t DV = 0;
  
  // FaultCode 10 bit binary representation of C
  uint16_t OVERVOLTAGE_WARNING = 0;
  uint16_t OVERVOLTAGE_CRITICAL = 0;  
  uint16_t LOWVOLTAGE_WARNING = 0;
  uint16_t LOWVOLTAGE_CRITICAL = 0; 
  uint16_t OVERTEMP_WARNING = 0;
  uint16_t OVERTEMP_CRITICAL = 0;
  uint16_t OVERDIV_VOLTAGE_WARNING = 0 ; // Div voltage is not a fault, it is to trigger balance
  uint16_t OVERDIV_VOLTAGE_CRITICAL = 0; // Critical voltage div will stop the charger for a time until the cell are properly balanced
  bool BMU_WARNING = 0;
  bool BMU_FAULTY = 0;
  // Status Binary Code
  uint16_t BalancingDischarge_Cells =0 ;
  uint8_t BMUOperationStatus = 0;
  
  bool BMUActive = 1; // Default as Active true , indicates it is active on the bus? or active as working, and monitoring
  bool BMUTimeout = 1; // if no ID received fomr this module , display in gui that it disconnected from the bus
  // Reset call
  BMSdata() {
        memset(V_CELL, 0, sizeof(V_CELL));
        memset(TEMP_SENSE, 0, sizeof(TEMP_SENSE));
        } 
} BMS_ROSPackage[BMU_NUM] ; 
// Basic OBC Data // Also
uint8_t OBCVolt = 0;
uint8_t OBCAmp = 0;
uint8_t OBCstatusbit = 0 ;   // During Charging

// constant

// Timers
unsigned long reference_time = 0;
unsigned long reference_time2 = 0;
unsigned long reference_time3 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;

// Flag
const bool EEPROM_WRITE_FLAG = false; // Manually Change this to true for updating Default Parameter
bool &SDC_OK_SIGNAL = SDCstat.SHUTDOWN_OK_SIGNAL; 
bool &BMS_OK = SDCstat.BMS_OK;
bool &IMD_OK = SDCstat.IMD_OK; 
bool &BSPD_OK = SDCstat.BSPD_OK; 
bool CHARGING_FLAG = false; // (Will use Hardware Interrupt later)
bool CAN_TIMEOUT_FLAG = false;

unsigned int ACCUM_MAXVOLTAGE = VMAX_CELL * CELL_NUM;
byte  VmaxCell ,VminCell ,TempMaxCell ,dVmax;

//Class or Struct for Data logging ------------------------*****************------------------*//
/*--------------------------------------------------------------------------------------------*/

/*******************************************************************
  Setup Program
********************************************************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  // pinMode(SDCIN,INPUT_PULLUP); // edit this thing out when real use 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(BMSOUT,OUTPUT);
  // In case of specific troubleshooting
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  
  /* Write Default Parameter to EEPROM only once*/
    EEPROM.begin(EEPROM_SIZE); // only for esp32 , that use virtual eeprom
    if (EEPROM_WRITE_FLAG) {
      Serial.println("write to EEPROM.");
      EEPROM.write(0, (byte) VMAX_CELL / 0.1 ); // 42
      EEPROM.write(1, (byte) VMIN_CELL / 0.1 ); // 32
      EEPROM.write(2, (byte) TEMP_MAX_CELL ); // 60
      EEPROM.write(3, (byte) DVMAX / 0.1 ); // 2
      EEPROM.write(4,CELL_NUM);
      EEPROM.write(5,BMU_NUM);
      EEPROM.commit();
      Serial.println("Writing to EEPROM. complete");
    }

    // Display and Check EEPROM Data
    Serial.println("EEPROM Data:");
    EEPROM.get(0,VmaxCell); Serial.println(VmaxCell);
    EEPROM.get(1,VminCell); Serial.println(VminCell);
    EEPROM.get(2,TempMaxCell); Serial.println(TempMaxCell);
    EEPROM.get(3,dVmax); Serial.println(dVmax);
    
    
  /* CAN Communication Setup */
    CAN.setPins(DEFAULT_CAN_RX_PIN,DEFAULT_CAN_TX_PIN);
    // CAN.filterExtended(0b00001100111011010000000000000101 , 0b11110000111111110000000011111111);
    // CAN.filterExtended(0x0CED0005 , 0xF0FF00FF);
    //  for SlaveNode filter to only pickup 0x0CE0E500 , no mask
    if (!CAN.begin(STANDARD_BITRATE)) {
      Serial.println("Starting CAN failed!");
      while(1);
    }
    Serial.println("__BCU Initialized__");
}

/*******************************************************************
  Routine Program
********************************************************************/

void loop(){

  /*---------------------------------------------Check Fault , Shutdown*/ 
    // //  Read SHUTDOWN_OK_SIGNAL signal according to the actual physical voltage of SDC
    // (digitalRead(SDCIN)) ? (SDCstat.SHUTDOWN_OK_SIGNAL = 1) : (SDCstat.SHUTDOWN_OK_SIGNAL = 0);

    // Read IMD_Ok and BSPD_OK status from shutdown circuit
    (digitalRead(IMDIN)) ? (IMD_OK = 1) : (IMD_OK = 0);
    (digitalRead(BSPDIN)) ? (BSPD_OK = 1) : (BSPD_OK = 0);
    
    // Confirm if the Charger is actually plugged (May change to interrupt)
    (digitalRead(OBCIN)) ? (CHARGING_FLAG = true) : (CHARGING_FLAG = false);


    // case 0 : communication timeout, via wiring or protocol error , lock MCU in this condition loop
    if( CAN_TIMEOUT_FLAG ) {
      // Reset the entired structure
      // for(short i =0 ; i <BMU_NUM ; i++)
      //   BMS_ROSPackage[i] = BMSdata(); 
      digitalWrite(BMSOUT,LOW);
      if( millis()-shutdown_timer1 >= SYNC_TIME*2){
        Serial.println("CANBUS_INACTIVE");
        shutdown_timer1 = millis();
      }
    }
    // when fault code detected, yes this works , but when receiving with cell data msg, why sometimes the light blink?
    // case 1: BMS detect fault , BMS_OK output LOW , while SHUTDOWN_OK_SIGNAL either read HIGH or LOW.
    else if ( !BMS_OK ) {

      digitalWrite(BMSOUT,LOW); 
      if( millis()-shutdown_timer1 >= SYNC_TIME*2){
        Serial.println("BMS_OK = 0");
        shutdown_timer1 = millis();
      }
    }
    // case 3 , Both BMS and SDC operate Normally , BMS output HIGH & SHUTDOWN_OK_SIGNAL read HIGH
    else if( (BMS_OK) && (SDC_OK_SIGNAL) ) {

      digitalWrite(BMSOUT,HIGH);
      // Reaffirm BMS_OK & SHUTDOWN_OK_SIGNAL as Normal Operation
      BMS_OK = 1;
      SDC_OK_SIGNAL = 1;
    } 
    // May be this case isn't needed
    // case 3: Fault due to other system , BMS_OK HIGH , SHUTDOWN_OK_SIGNAL read LOW , IMD , and BSPD may read HIGH or low
    else {
      // digitalWrite(BMSOUT,LOW); 
      if( millis()-shutdown_timer1 >= SYNC_TIME*2){
        Serial.println("SHUTDOWN DUE TO OTHER SYSTEM");
        shutdown_timer1 = millis();
      }
    }
    

  /*--------------BMS_OK check-------------*/
    (OBCstatusbit > 0) ? (BMS_OK = 0): (BMS_OK = 1);
    // Faulting Matrix , looping to check : (May need to improve to a more time efficient method)
  
    bool FAULTCHECKSUM;
    // OR all fault condition per module , if 1 appear,  means overall BMU is faulty
    for(short i =0; i < BMU_NUM ; i++){

      // Compile the warning flag , to signal GUI that specific module at CRITICAL
      BMS_ROSPackage[i].BMU_FAULTY = BMS_ROSPackage[i].OVERVOLTAGE_CRITICAL | BMS_ROSPackage[i].LOWVOLTAGE_CRITICAL 
                                    | BMS_ROSPackage[i]. OVERTEMP_CRITICAL;
      
      
      // Compile the warning flag , to signal GUI that specific module at WARNING
      BMS_ROSPackage[i].BMU_WARNING = BMS_ROSPackage[i].OVERVOLTAGE_WARNING | BMS_ROSPackage[i].LOWVOLTAGE_WARNING 
                                    | BMS_ROSPackage[i]. OVERTEMP_WARNING;
    }

    // IF ANY of the Module is at Critical Raise a Fault flag, BMS_OK = 0;
    FAULTCHECKSUM = BMS_ROSPackage[0].BMU_FAULTY | BMS_ROSPackage[1].BMU_FAULTY | BMS_ROSPackage[2].BMU_FAULTY |
                    BMS_ROSPackage[3].BMU_FAULTY | BMS_ROSPackage[4].BMU_FAULTY | BMS_ROSPackage[5].BMU_FAULTY |
                    BMS_ROSPackage[6].BMU_FAULTY | BMS_ROSPackage[7].BMU_FAULTY;
    (FAULTCHECKSUM > 0) ? (BMS_OK = 0) : (BMS_OK = 1);

    // Dealing with warning flag
    // Dealing with OVERDIV VOLTAGE
    // DIV voltage During Driving --
      //WARNING
      //CRITICAL
      // Do nothing
    // DIV voltage during charging --
      //WARNING
      //CRITICAL

  /*------------------------------------------Message Transmission Routine*/ 

    // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 
    if( millis()-reference_time >= (SYNC_TIME/2)){
      BCUtoBMU_cmd_messagePrep(&sendmsg);
      CANsend(&sendmsg); // Broadcast to BMU
      reference_time = millis();
    }
    // BCU => OBC       (500ms cycle) , only when CHARGING_FLAG = TRUE
    if( millis()-reference_time2 >= OBC_SYNC_TIME && CHARGING_FLAG ) {
      BCUtoOBC_cmd_messagePrep(&sendmsg,);
      CANsend(&sendmsg);
      reference_time2 = millis();
    }

/*------------------------------------------Message Reception Routine*/

  // Polling for Monitoring Messages
  if (CAN.parsePacket() || CAN.peek() != -1) {
    
    // Receiving the message : 200ms Cycle,  1000ms cycle
    CANreceive(&receivemsg);
    // --------------Basic Debugging (takes about <1ms)
    Serial.println(receivemsg.can_id , HEX);
    // for(short i = 0 ; i < receivemsg.can_dlc  ; i++) {
    //   Serial.print(receivemsg.data[i] , HEX);
    // } Serial.println();
    // checkModuleTimeout();
    // how to check if individual module timeout, check only for the src address part , if none received within Timeout = 4s
    // then that module isn't on the bus (due to CAN error , or its own error) (But this system is built to not having fault)

    // Pack data in BMS_ROSPackage => 
    // ----Driving
    BCUmonitorBMU(&receivemsg); // 200ms cycle & 1000ms cycle of faultcode
    // ----Charging
    if(CHARGING_FLAG)
      BCUmonitorOBC(&receivemsg); // 500ms cycle
    
    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLAG = false;
    communication_timer1 = millis();

  } 
  // If receiving 0 bytes from CAN Bus check communication_timer >= TIMEOUT_TIME
  else if (millis()-communication_timer1 >= TIMEOUT_TIME){
    CAN_TIMEOUT_FLAG = true;
  }
  
  /*-------------- BMSROS_Package debug with SD card logger -------------*/

  // status bit might need conversion to array in order to print easily
  
  
  /*-------------- CORE 0 Tasks -------------*/
  // ROS , Ethernet publishy function
} 

 

/*******************************************************************
  Local Functions Definition
********************************************************************/

void BCUtoBMU_cmd_messagePrep ( _can_frame* BCUsent) {

  BCUsent->can_id  = 0x10E0E500; // BCU ID
  BCUsent->can_dlc = 8;
  
  // BMU synchronize time
  BCUsent->data[0] = SYNC_TIME;
  // Balance Active Command
  (CHARGING_FLAG) ? (BCUsent->data[1] = 1) : (BCUsent->data[1] = 0);
  
  // Distribute Default parameter
  BCUsent->data[2] = VmaxCell; 
  BCUsent->data[3] = VminCell; 
  BCUsent->data[4] = TempMaxCell; 
  BCUsent->data[5] = dVmax;
  BCUsent->data[6] = 0x00; // Reserved
  BCUsent->data[7] = 0x00;
    
}


void BCUmonitorBMU ( _can_frame* BCUreceived) {
  
  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->can_id));
  // Do not monitor any messsage that is not associate with BMS
  if(decodedCANID.BASE_ID != 0x0E && decodedCANID.DEST != 0xE5)
    return;
  
  byte i;
  switch (decodedCANID.SRC)
  {
  case 0x01: i = 0; break;
  case 0x02: i = 1; break;
  case 0x03: i = 2; break;
  case 0x04: i = 3; break;
  case 0x05: i = 4; break;
  case 0x06: i = 5; break;
  case 0x07: i = 6; break;
  case 0x08: i = 7; break;
  }


  /*  Message Priority 11 :: BMUModule & Cells data  */
  if(decodedCANID.PRIORITY == 0x11){
    // Sort out the cell message
    switch (decodedCANID.MSG_NUM) 
    { 
      // MSG1 == Operation status
      case 1:
        // Operation status
        BMS_ROSPackage[i].BMUOperationStatus = BCUreceived->data[0];
        // Balancing Discharge cell number
        BMS_ROSPackage[i].BalancingDischarge_Cells = mergeHLbyte(BCUreceived->data[1],BCUreceived->data[2]);
        // Vbatt (Module) , dVmax(cell)
        BMS_ROSPackage[i].V_MODULE = BCUreceived->data[3]; 
        BMS_ROSPackage[i].DV = BCUreceived->data[4]; 
        // Temperature sensor
        BMS_ROSPackage[i].TEMP_SENSE[0] = BCUreceived->data[5];
        BMS_ROSPackage[i].TEMP_SENSE[1] = BCUreceived->data[6];
        
        break;

      case 2:
        // Low series Side Cell C1-C7
        // 2 loop or ?? this isn't good O(n^2)
        for(short j=0; j< 8; j++)
          BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i]*0.02;

        break;

      case 3:
        // High series side Cell C8-C10
        for( short j = 8; j <= 15; j++ )
          BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i-8]*0.02;
        break;
    }
  }

  /*  Message Priority 10 :: FaultCode  */
  /*   Once Critical state is reached , warning stop , and the Car Shutdown Immediately   */
  else if(decodedCANID.PRIORITY == 0x10){
    // Aggregate data to ROS topics , display which specific cell is at fault
    switch (decodedCANID.MSG_NUM)
    {
      case 1:

        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMS_ROSPackage[i].OVERVOLTAGE_WARNING =  mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
        BMS_ROSPackage[i].OVERVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] );  
        BMS_ROSPackage[i].LOWVOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] );
        BMS_ROSPackage[i].LOWVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ); 
        break;
      case 2:
       
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMS_ROSPackage[i].OVERTEMP_WARNING = mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
        BMS_ROSPackage[i].OVERTEMP_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] ); 
        BMS_ROSPackage[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] ); 
        BMS_ROSPackage[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] );
        break;
    }
    
  }
}

void checkModuleTimeout(){
  // CANIDDecoded decodedCANID;
  // decodeExtendedCANID(&decodedCANID,receivemsg.can_id);
  //   if(decodedCANID.SRC != 0x01){
        
  //   } else if
  //   // else if until one condition is true

}

// also for the case of Diff Voltage, the div voltage should not trigger The OK condition
void BCUtoOBC_cmd_messagePrep ( _can_frame* BCUsent, bool cmd ) {

  /* Set up BMS CAN frame*/
  BCUsent->can_id  = 0x1806E5F4; // refers to specification sheet
  BCUsent->can_dlc = 8;
  // Reserved
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;

  // Condition 1 iF BMS_OK Or manual cmd difference voltage commands,
  if(BMS_OK || cmd) {
    BCUsent->data[0] = 0x0C; // V highbyte 
    BCUsent->data[1] = 0x80; // V lowbyte 72.0 V fake data -> Range 69-72-74 V
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

}


void BCUmonitorOBC ( _can_frame* BCUreceived ) {
  // if message ID isnt 0x18FF50E5 , return
  if(BCUreceived->can_id != 0x18FF50E5)
    return;
    // Monitor & Translate current Frame data
    uint8_t VoutH = BCUreceived->data[0];
    uint8_t VoutL = BCUreceived->data[1];
    uint8_t AoutH = BCUreceived->data[2];
    uint8_t AoutL = BCUreceived->data[3];
    OBCstatusbit =  BCUreceived->data[4]; // Status Byte
    OBCVolt = mergeHLbyte(VoutH,VoutL);
    OBCAmp = mergeHLbyte(AoutH,AoutL);
}


// Recording to local data logger

void debugBMUmsg(){

    // Serial.print("BMU Operation Status: "); Serial.println(BMUOperationStatus);
    // Serial.print("Cell balancing discharge: "); Serial.println(BalancingDischarge_Cells);
    // Serial.print("V_CELL C1-10: ");
    // for(short i=0; i< CELL_NUM; i++){
    //   Serial.print(V_CELL[i]); Serial.print("V.  ");
    // } Serial.println();
    // Serial.print("V_MODULE: "); Serial.print(V_MODULE); Serial.println("V.  ");
    // Serial.print("AVG_CELL_VOLTAGE_DIFF: ") ; Serial.print(DV); Serial.println("V.  ");

    // Serial.print("TEMP_SENSOR T1-T2: ");
    // for(short i=0; i< TEMP_SENSOR_NUM; i++){
    //   Serial.print(TEMP_CELL[i]); Serial.print("C.  ");
    // } Serial.println();

    // Serial.println();

}
void debugBMUFault(){
  // if(!BMS_OK){
  //   Serial.print("OVERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
  //   Serial.println(OVERVOLTAGE_CRITICAL,HEX);
  //   Serial.print("UNDERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
  //   Serial.println(LOWVOLTAGE_CRITICAL,HEX);
  //   Serial.print("OVERTEMP_CRITICAL (C1-C10): ");
  //   Serial.println(OVERTEMP_CRITICAL, HEX);
  //   Serial.print("OVERDIV_CRITICAL (C1-C10): ");
  //   Serial.println(OVERDIV_VOLTAGE_CRITICAL,HEX);
  //   Serial.println();
  // } else {
    
  // }
    // Serial.print("OVERVOLTAGE_WARNING_CELLS (C1-C10): ");
    // Serial.println(OVERVOLTAGE_WARNING,HEX);
    // Serial.print("UNDERVOLTAGE_WARNING_CELLS (C1-C10): ");
    // Serial.println(LOWVOLTAGE_WARNING,HEX);
    // Serial.print("OVERTEMP_WARNING (C1-C10): ");
    // Serial.println(OVERTEMP_WARNING, HEX);
    // Serial.print("OVERDIV_WARNING (C1-C10): ");
    // Serial.println(OVERDIV_VOLTAGE_WARNING,HEX);
    // Serial.println();  
    
}

void debugOBCmsg(){
    
    // Serial.print("Voltage from OBC: "); Serial.print(OBCVolt); Serial.println("V");
    // Serial.print("Current from OBC: "); Serial.print(OBCAmp); Serial.println("A");
    // Serial.print("OBC status bit"); Serial.println(OBCstatusbit);

    // // Intepret Individual bit meaning
    // bool *obcstatbitarray =  toBitarrayLSB(OBCstatusbit); // Status Byte
    
    // Serial.print("OBC status bit: ");
    // switch (obcstatbitarray[0]) {
    //   case 1:
    //     Serial.println("ChargerHW = Faulty");
    //     break;
    // }
    // switch (obcstatbitarray[1]) {
    //   case 1:
    //     Serial.println("ChargerTemp = Overheat");
    //     break;
    // }
    // switch (obcstatbitarray[2]) {
    //   case 1:
    //     Serial.println("ChargerACplug = Reversed");
    //     break;
    // }
    // switch (obcstatbitarray[3]) {
    //   case 1:
    //     Serial.println("Charger detects: NO BATTERY VOLTAGES");
    //     break;
    // }
    // switch (obcstatbitarray[4]) {
    //   case 1:
    //     Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
    //     break;
    // } 
}
// Yuil sketch function to publish ROS topics