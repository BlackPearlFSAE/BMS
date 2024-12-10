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

/************************* Define macros *****************************/
#define SDCIN 9   // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty SDC
#define OBCIN 10   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
#define BSPDIN 13  // Check BSPD OK signal from BSPD directly 
#define IMDIN 14   // Check IMD OK signal from IMD directly
#define BMSOUT LED_BUILTIN   // OUTPUT Fault Signal to BMS relay
#define EEPROM_SIZE 5
#define SHUTDOWN_MSG_PERIOD 200
#define BCU_CMDMSG_PERIOD 100
#define SYNC_TIME 200

#define VMAX_CELL 4.2
#define VMIN_CELL 3.2
#define TEMP_MAX_CELL 60
#define DVMAX 0.2
#define CELL_NUM 10
#define BMU_NUM 8
/**************** Local Function Delcaration *******************/

void BCUtoOBC_cmd_messagePrep ( _can_frame* BCUsent );
void BCUmonitorOBC ( _can_frame* BCUreceived );
void BCUtoBMU_cmd_messagePrep ( _can_frame* BCUsent);
void BCUmonitorBMUcell ( _can_frame* BCUreceived );
void BCUinterpretBMUfaultcode(_can_frame* BCUreceived);

/**************** Setup Variables *******************/

_can_frame sendmsg;
_can_frame receivemsg;

// Maybe I'll change them to typedef in case of C compatibility??

// Interacti with SDC and Physical Circuit
struct SDCstatus {
  bool SHUTDOWN_OK_SIGNAL = 1; 
  bool BMS_OK = 1;
  bool IMD_OK = 1;
  bool BSPD_OK = 1;
  //  AMS active light
  // TSAL??
  // cockpit switch
} SDCstat;

// Interact with Telemetry system
struct BMSdata {

  float V_CELL[CELL_NUM]; //  for now last 4 cell is not connected
  float TEMP_CELL[2];
  float V_MODULE;
  float DV;
  // FaultCode 10 bit binary representation of C
  uint16_t OVERVOLTAGE_WARING;
  uint16_t OVERVOLTAGE_CRITICAL;  
  uint16_t LOWVOLTAGE_WARNING; 
  uint16_t LOWVOLTAGE_CRITICAL; 
  uint16_t OVERTEMP_WARNING; 
  uint16_t OVERTEMP_CRITICAL;
  uint16_t OVERDIV_VOLTAGE_WARNING; // Not fault
  uint16_t OVERDIV_VOLTAGE_CRITICAL;
  uint16_t BalancingDischarge_Cells;
  uint8_t BMUOperationStatus;
} BMS_ROSPackage ;

// Constant


// Timers
unsigned long reference_time = 0;
unsigned long reference_time2 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;

// Flag
const bool EEPROM_WRITE_FLAG = false; // Manually Change this to true for updating Default Parameter
bool &SDC_OK_SIGNAL = SDCstat.SHUTDOWN_OK_SIGNAL; 
bool &BMS_OK = SDCstat.BMS_OK;
bool &IMD_OK = SDCstat.IMD_OK; 
bool &BSPD_OK = SDCstat.BSPD_OK; 
bool CHARGING_FLAG = false; // (May use External Interrupt to change this flag later)
// FaultCode
uint16_t &OVERVOLTAGE_WARNING = BMS_ROSPackage.OVERDIV_VOLTAGE_WARNING;
uint16_t &OVERVOLTAGE_CRITICAL = BMS_ROSPackage.OVERDIV_VOLTAGE_CRITICAL;  
uint16_t &LOWVOLTAGE_WARNING = BMS_ROSPackage.LOWVOLTAGE_WARNING; 
uint16_t &LOWVOLTAGE_CRITICAL = BMS_ROSPackage.LOWVOLTAGE_CRITICAL; 
uint16_t &OVERTEMP_WARNING = BMS_ROSPackage.OVERTEMP_WARNING; 
uint16_t &OVERTEMP_CRITICAL = BMS_ROSPackage.OVERTEMP_CRITICAL;
uint16_t &OVERDIV_VOLTAGE_WARNING = BMS_ROSPackage.OVERDIV_VOLTAGE_WARNING;
uint16_t &OVERDIV_VOLTAGE_CRITICAL = BMS_ROSPackage.OVERDIV_VOLTAGE_CRITICAL;

bool CAN_TIMEOUT_FLAG = false;

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
    CAN.filterExtended(0x0CED0005 , 0xF0FF00FF);
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
    //  Read SHUTDOWN_OK_SIGNAL signal according to the actual physical voltage of SDC
    // (digitalRead(SDCIN) ? (SDCstat.SHUTDOWN_OK_SIGNAL = 1) : (SDCstat.SHUTDOWN_OK_SIGNAL = 0);

    // Read IMD_Ok and BSPD_OK status from shutdown circuit
    (digitalRead(IMDIN)) ? (IMD_OK = 1) : (IMD_OK = 0);
    (digitalRead(BSPDIN)) ? (BSPD_OK = 1) : (SDCstat.BSPD_OK = 0);
    
    // Confirm if the Charger is actually plugged 
    (digitalRead(OBCIN)) ? (CHARGING_FLAG = true) : (CHARGING_FLAG = false);


    // case 0 : communication timeout, via wiring or protocol error , lock MCU in this condition loop
    if( CAN_TIMEOUT_FLAG == true ) {

      digitalWrite(BMSOUT,LOW);
      if( millis()-shutdown_timer1 >= SHUTDOWN_MSG_PERIOD ){
        Serial.println("BCU detected Communication Timeout");
        shutdown_timer1 = millis();
      }
    }
    
    // case 1: BMS detect fault , BMS_OK output LOW , while SHUTDOWN_OK_SIGNAL either read HIGH or LOW.
    else if ( BMS_OK == 0 ) {

      digitalWrite(BMSOUT,LOW); 
      if( millis()-shutdown_timer1 >= SHUTDOWN_MSG_PERIOD ){
        Serial.println("BMS_COMMAND_SHUTDOWN!");
        shutdown_timer1 = millis();
      }
    }
    // case 3 , Both BMS and SDC operate Normally , BMS output HIGH & SHUTDOWN_OK_SIGNAL read HIGH
    else if( (BMS_OK == 1) && (SDC_OK_SIGNAL == 1) ) {

      digitalWrite(BMSOUT,HIGH);
      // Reaffirm BMS_OK & SHUTDOWN_OK_SIGNAL as Normal Operation
      BMS_OK = 1;
      SDC_OK_SIGNAL = 1;
    } 

    // case 3: Fault due to other system , BMS_OK HIGH , SHUTDOWN_OK_SIGNAL read LOW
    else {
      digitalWrite(BMSOUT,LOW); 
      if( millis()-shutdown_timer1 >= SHUTDOWN_MSG_PERIOD){
        Serial.println("OTHERSYSTEM_COMMAND_SHUTDOWN");
        shutdown_timer1 = millis();
      }
    }
    

    // Prepare CAN frame , then send
    // Receive CAN frame , then interpret

  /*---------------------------------------------Message Transmission Routine*/ 

    // BCU CMD & SYNC message (100ms cycle Broadcast to all BMU modules in Bus) 
    if( millis()-reference_time >= BCU_CMDMSG_PERIOD){

      BCUtoBMU_cmd_messagePrep(&sendmsg);
      CANsend(&sendmsg); // Broadcast to BMU
      
      reference_time = millis();
    }

    // BCU OBC Communication (500ms cycle) , only when CHARGING_FLAG = TRUE
    if( millis()-reference_time2 >= 500 && CHARGING_FLAG ) {

      BCUtoOBC_cmd_messagePrep(&sendmsg);
      CANsend(&sendmsg);

      reference_time2 = millis();
    }

/*---------------------------------------------Message Reception Routine*/
  // Polling for Monitoring Messages
  if (CAN.parsePacket() || CAN.peek() != -1) {

    CANreceive(&receivemsg);
    BCUmonitorBMUcell(&receivemsg);

    // For Charging Event , additonally interpret the message as OBC as well
    if(CHARGING_FLAG)
      BCUmonitorOBC(&receivemsg);
    
    // Update timeout flag and Communication timer
    CAN_TIMEOUT_FLAG = false;
    communication_timer1 = millis();
  } 
  // If receiving 0 bytes check communication timer
  else if (millis()-communication_timer1 >= 4000){
    CAN_TIMEOUT_FLAG = true;
  }

  // BMU Module Report , BMU individual cell report (2 monitoring message per module) (200ms cycle)
    // Message reception sequencing :: 200 ms / 8  = 25 ms / module
    // Each module will have its own algorithm to calculate the time : eg. 200ms/8 * src address ,src add = 0x01-0x08
    
    /* Print BMSROS_Package to check the data*/
    Serial.print("BMU Operation Status"); Serial.print(":");
    Serial.print("Cell balancing discharge"); Serial.print(":");
    // Serial.print(V_cell[i]); Serial.print("V.   ");

    // Serial.print("Temp"); Serial.println(":");
    // Serial.print(Temp_cell[0]); Serial.println("C");
    // Serial.print(Temp_cell[1]); Serial.println("C");
    // Serial.print(Temp_cell[2]); Serial.println("C");

    // Display Faultcode arrays (Display Both warning and Critical State)
    Serial.print("OVERVOLTAGE_CELLS (C1-C16): ");
    Serial.println(OVERDIV_VOLTAGE_WARNING);
    Serial.print("UNDERVOLTAGE_CELLS (C1-C16): ");
    Serial.println(LOWVOLTAGE_WARNING);
    Serial.print("OVERTEMP_WARNING (C1-C16): ");
    Serial.print(OVERTEMP_WARNING);
    Serial.print("OVERDIV_WARNING (C1-C16): ");
    Serial.println(OVERDIV_VOLTAGE_WARNING);
  
    // Critical Zone , show once reach...

    

    Serial.print("Low Series Cell C1-C8"); Serial.print(":");
    Serial.print("High Series Cell C9-C16"); Serial.print(":");

    // ROS , Ethernet publishy function

} 
 

/*******************************************************************
  Local Functions Definition
********************************************************************/

void BCUtoBMU_cmd_messagePrep ( _can_frame* BCUsent) {
    
  /* Set up BMS CAN frame*/
    // BCUsent->can_id  = createExtendedCANID(0x00,0xCE,0x00,0xE5,0x00);
    BCUsent->can_id  = 0x0CE0E00; // BCU ID
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


void BCUmonitorBMUcell ( _can_frame* BCUreceived ) {
  
  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->can_id));

  
  // Do not interpret any messsage that is not associate with BMS
  if(decodedCANID.BASE_ID != 0xCE && decodedCANID.DEST != 0xE5)
    return;
  
  float *V_cell = BMS_ROSPackage.V_CELL; //  for now last 4 cell is not connected
  float *Temp_cell = BMS_ROSPackage.TEMP_CELL;
  float &V_Module = BMS_ROSPackage.V_MODULE;
  float &dV = BMS_ROSPackage.DV;
  uint8_t &BMUOperationStatus = BMS_ROSPackage.BMUOperationStatus ; // 8 bit
  uint16_t &BalancingDischarge_Cells = BMS_ROSPackage.BalancingDischarge_Cells; // 10 bit
  
  // ---Priority 0 messsage
  if(decodedCANID.PRIORITY == 0){

    // Operation status
    BMUOperationStatus = BCUreceived->data[0];

    // Balancing Discharge cell number
    
    BalancingDischarge_Cells = mergeHLbyte(BCUreceived->data[1],BCUreceived->data[2]);
    
      // Vbatt (Module) , dVmax(cell)
    V_Module = BCUreceived->data[3]*0.2; 
    dV = BCUreceived->data[4]*0.1; 

    // Temperature sensor
    Temp_cell[0] = 2 + BCUreceived->data[5]*0.0125;
    Temp_cell[1] = 2 + BCUreceived->data[6]*0.0125;
    Temp_cell[2] = 2 + BCUreceived->data[7]*0.0125;
    
  }
  else if(decodedCANID.PRIORITY == 1) {

    switch (decodedCANID.MSG_NUM) {
      
      case 1:
        // Low series Side Cell
          
          for( short i = 1; i <= 7; i++ ){
            V_cell[i] = BCUreceived->data[i]*0.02;
          } 
          Serial.println();
        
        break;
      
      case 2:
        // High series side Cell
          
          for( short i = 1; i <= 7; i++ ) {
            V_cell[i] = BCUreceived->data[i]*0.02;
          } 
          Serial.println();
        
        break;
    }

  }
}

// BMS read SDC function
void BCUinterpretBMUfaultcode (_can_frame* BCUreceived){

  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->can_id));

  if(decodedCANID.PRIORITY != 0x02 && decodedCANID.BASE_ID != 0xCE && decodedCANID.DEST != 0xE5)
    return;

  /*  Message Priority 2 :: FaultCode  */
  /*   Once Critical state is reached , warning stop , and the Car Shutdown Immediately   */

  uint16_t overvoltage_warning;
  uint16_t overvoltage_critical;  
  uint16_t lowvoltage_warning; 
  uint16_t lowvoltage_critical; 
  uint16_t overtemp_warning; // Not fault
  uint16_t overtemp_critical; 
  uint16_t overdivVoltage_warning; // Not fault
  uint16_t overdivVoltage_critical;

  
  switch (decodedCANID.MSG_NUM)
  {
    case 1:
      // Merge H and L byte of each FaultCode back to 10 bit binary
      overvoltage_warning = mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
      overvoltage_critical = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] ); 
      lowvoltage_warning =  mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] ); 
      lowvoltage_critical = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ); 

      // Aggregate data to ROS topics , display which specific cell is at fault
      OVERVOLTAGE_WARNING = overvoltage_warning;  
      OVERDIV_VOLTAGE_CRITICAL = overvoltage_critical; 
      LOWVOLTAGE_WARNING = lowvoltage_warning; 
      LOWVOLTAGE_CRITICAL = lowvoltage_critical;

      break;
    
    case 2:
      // Merge H and L byte of each FaultCode back to 10 bit binary
      overtemp_warning = mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
      overtemp_critical = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] ); 
      overdivVoltage_warning =  mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] ); 
      overdivVoltage_warning = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ); 

      // Aggregate data to ROS topics , display which specific cell is at fault
      OVERTEMP_WARNING = overtemp_warning;  
      OVERTEMP_CRITICAL = overtemp_critical; 
      OVERDIV_VOLTAGE_WARNING = overdivVoltage_warning; 
      OVERDIV_VOLTAGE_CRITICAL = overdivVoltage_warning;
      break;
  }
  
  
  // Condition check, if the bitstatus > 0 , it contains 1 , which means one of the cell is at fault
  // which will makes faultmatrix = 1 if all condition went through OR
  bool faultmatrix = OVERVOLTAGE_CRITICAL | LOWVOLTAGE_CRITICAL | OVERTEMP_CRITICAL | OVERDIV_VOLTAGE_CRITICAL;
    (faultmatrix == 1) ? (BMS_OK = 0) : (BMS_OK = 1); 
  // bool warningmatrix = OVERVOLTAGE_WARNING | LOWVOLTAGE_WARNING | OVERTEMP_WARNING | OVERDIV_VOLTAGE_WARNING;
  // Warning Display function (just in Serial monitor , in practical needs to be in GUI)
  
  
}


void BCUtoOBC_cmd_messagePrep ( _can_frame* BCUsent ) {
  // There needs to be a 1st message to make the OBC not entering COMMUNICATION ERROR
    /* Set up BMS CAN frame*/
    BCUsent->can_id  = 0x1806E5F4; // refers to specification sheet
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


void BCUmonitorOBC ( _can_frame* BCUreceived ) {
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
        uint16_t obcstatusbit =  BCUreceived->data[4]; // Status Byte
        // Fault Check if Status bit contains one , or > 0 , BMS_NOTOK
        (obcstatusbit >0) ? (BMS_OK = 0): (BMS_OK = 1);


      bool *obcstatbitarray =  toBitarrayLSB(obcstatusbit); // Status Byte
      // Intepret Individual bit meaning
      Serial.print("OBC status bit: ");
      switch (obcstatbitarray[0]) {
        case 1:
          Serial.println("ChargerHW = Faulty");
          break;
      }
      switch (obcstatbitarray[1]) {
        case 1:
          Serial.println("ChargerTemp = Overheat");
          break;
      }
      switch (obcstatbitarray[2]) {
        case 1:
          Serial.println("ChargerACplug = Reversed");
          break;
      }
      switch (obcstatbitarray[3]) {
        case 1:
          Serial.println("Charger detects: NO BATTERY VOLTAGES");
          break;
      }
      switch (obcstatbitarray[4]) {
        case 1:
          Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
          break;
      } 
    } 
}




// Yuil sketch function to publish ROS topics