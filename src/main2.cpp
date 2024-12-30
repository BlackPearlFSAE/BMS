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
#include <driver/gpio.h>
#include <driver/twai.h>
#include "esp_log.h"
#include <Ticker.h>
// #include <freertos/FreeRTOS.h>
// #include <micro_ros_arduino.h>

// #include "FS.h"
// #include "SD.h"
// #include "SPI.h"
#include <Arduino.h>
#include <EEPROM.h>
// utility function
#include <util.h>

/************************* Define macros *****************************/
#define CAN_RX  GPIO_NUM_13
#define CAN_TX  GPIO_NUM_14
#define SDCIN GPIO_NUM_9    // Check OK signal from Shutdown Circuit => Its status must match all below Signal pin , otherwise faulty
#define OBCIN GPIO_NUM_10   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
#define BSPDIN GPIO_NUM_4  // Check BSPD OK signal from BSPD directly 
#define IMDIN GPIO_NUM_5   // Check IMD OK signal from IMD directly
#define BMSOUT GPIO_NUM_47  // OUTPUT Fault Signal to BMS relay
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

void packOBCmsg ( twai_message_t *BCUsent , bool cmd);
void packBMUmsg ( twai_message_t *BCUsent);
void unpackOBCmsg ( twai_message_t *BCUreceived );
void unpackBMUmsg ( twai_message_t *BCUreceived, BMSdata *BMS_ROSPackage);
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();
void debugSDC();


/**************** Setup Variables *******************/


twai_message_t sendMessage;
twai_message_t receivedMessage;

// struct _can_frame sendmsg;
// struct _can_frame receivemsg;

// Data Aggregation , and Relay to Other Sub System , e.g. Telemetry , DataLogger, BMS GUI
// Report BMS data, and Faulty status code // Serialize this data in 8 bit buffer

// BMU data strcuture
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
  uint16_t OVERDIV_VOLTAGE_WARNING = 0 ; // Trigger cell balancing of the cell at fault
  uint16_t OVERDIV_VOLTAGE_CRITICAL = 0; // Trigger Charger disable in addition to Cell balancing
  bool BMU_WARNING = 0;
  bool BMU_FAULTY = 0;
  bool BMUActive = 1;   // Default as Active true , means each BMU is on the bus
  
  // Status Binary Code
  uint16_t BalancingDischarge_Cells =0 ;
  uint8_t BMUOperationStatus = 0;
  
  // Reset call
  BMSdata() {
        memset(V_CELL, 0, sizeof(V_CELL));
        memset(TEMP_SENSE, 0, sizeof(TEMP_SENSE));
        } 
} BMS_ROSPackage[BMU_NUM]; 

// Physical condition of SDC and LV Circuit, safety information , and relay it to Telemetry system
struct SDCstatus {
  bool CAN_TIMEOUT = 0;
  bool SHUTDOWN_OK_SIGNAL = 1; // AIR+
  bool BMS_OK = 1; // AMS_OUT+ ,AMS_GND
  bool IMD_OK = 1; // IMD_OUT
  bool BSPD_OK = 1; // BSPD_OUT
} SDCstat;

// Physical condition of OBC On board charger , charging power , and safety information
uint8_t OBCVolt = 0;
uint8_t OBCAmp = 0;
uint8_t OBCstatusbit = 0 ;   // During Charging

// Software Timer reference
unsigned long reference_time = 0;
unsigned long reference_time2 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;
// Ticker , Timer interrupt
Ticker ticker1; // Debugging BMU Cell msg
Ticker ticker2; // Debugging BMU Fault code msg
Ticker ticker3; // Debugging OBC Fault code msg
Ticker ticker4; // Debugging OBC Fault code msg
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// Flag
const bool EEPROM_WRITE_FLAG =  false;
bool &SDC_OK_SIGNAL = SDCstat.SHUTDOWN_OK_SIGNAL; 
bool &BMS_OK = SDCstat.BMS_OK;
bool &IMD_OK = SDCstat.IMD_OK; 
bool &BSPD_OK = SDCstat.BSPD_OK; 
bool CHARGING_FLAG = false; // (Will use Hardware Interrupt later)
bool CAN_TIMEOUT_FLAG = false;
bool CAN_SEND_FLG1 = 0;
bool CAN_SEND_FLG2 = 0;
bool OVERDIV_FLAG = 0;

unsigned int ACCUM_MAXVOLTAGE = VMAX_CELL * CELL_NUM; // Expected
byte  VmaxCell ,VminCell ,TempMaxCell ,dVmax;


/*******************************************************************
  ==============================Setup==============================
********************************************************************/

void IRAM_ATTR onTimer1() {
  CAN_SEND_FLG1 = 1;
}
void IRAM_ATTR onTimer2() {
  CAN_SEND_FLG2 = 1;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  
  /* Shutdown System setup */
  pinMode(SDCIN,INPUT_PULLDOWN); 
  pinMode(OBCIN,INPUT_PULLDOWN);
  pinMode(BMSOUT,OUTPUT);
  // // In case of specific troubleshooting
  pinMode(IMDIN,INPUT_PULLDOWN);
  pinMode(BSPDIN,INPUT_PULLDOWN);
  
  /* Write Default Parameter to ESP32 Flashmemory only once*/
    EEPROM.begin(EEPROM_SIZE); // only for esp32 , that use virtual eeprom
    if (EEPROM_WRITE_FLAG) {
      Serial.println("write to EEPROM.");
      EEPROM.write(0, (byte) (VMAX_CELL / 0.1) ); // 42
      EEPROM.write(1, (byte) (VMIN_CELL / 0.1) ); // 32
      EEPROM.write(2, (byte) (TEMP_MAX_CELL) ); // 60
      EEPROM.write(3, (byte) (DVMAX / 0.1) ); // 2
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
  sendMessage.extd = true;
  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 800; // worstcase is 152 bit/frame , this should hold about 5 frame
  general_config.rx_queue_len = 1300; // RX queue hold about 8 frame

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
  // Set Filter (Default now us accept all , but I will reconfigure it later)
  twai_filter_config_t filter_config = {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true};

  // Install the TWAI driver
  if (twai_driver_install(&general_config, &timing_config, &filter_config) == !ESP_OK) {
    Serial.println("TWAI Driver install failed__");
    while(1);
  }
  // Start the TWAI driver
    if (twai_start() == ESP_OK) {
      Serial.println("TWAI Driver installed , started");
      /*
      TWAI_ALERT_RX_DATA        0x00000004    Alert(4)    : A frame has been received and added to the RX queue
      TWAI_ALERT_ERR_PASS       0x00001000    Alert(4096) : TWAI controller has become error passive
      TWAI_ALERT_BUS_ERROR      0x00000200    Alert(512)  : A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
      TWAI_ALERT_RX_QUEUE_FULL  0x00000800    Alert(2048) : The RX queue is full causing a frame to be lost
      */
      // Reconfigure the alerts to detect the error of frames received, Bus-Off error and RX queue full error
      // Configure Alert flag , which alert or Error can be raised
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == !ESP_OK) {
        Serial.println("Failed to reconfigure alerts");
        while(1);
      }
    }


    //  Timer interrupt (From Hal) for tasks
    // Setup timer for 100ms intervals
      My_timer1 =  timerBegin(0, 80, true);  // Timer with prescaler
      timerAttachInterrupt(My_timer1, onTimer1, true);
      timerAlarmWrite(My_timer1, 100000, true);  // 100ms interval
      timerAlarmDisable(My_timer1);

      // Setup timer for 500ms intervals 
      My_timer2 = timerBegin(0, 80, true);
      timerAttachInterrupt(My_timer2, onTimer2, true);
      timerAlarmWrite(My_timer2, 500000, true);  // 500ms interval
      timerAlarmDisable(My_timer2);

    // Ticker for debugging Set all the Ticker for debugging via serial monitor (Just Module 0)
      // ticker1.attach_ms( (SYNC_TIME) , debugBMUmsg, 0);
      // ticker2.attach_ms( (SYNC_TIME * 5) ,debugBMUmsg, 0); 
      // ticker3.attach_ms( (OBC_SYNC_TIME) , debugOBCmsg);
      // ticker4.attach_ms( (SYNC_TIME), debugSDC);

    Serial.println("BCU__initialized__");
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/

void loop(){

    /* ==================================================== Task 3 : Communication ====================================================*/
  /*------------------------------------------Message Transmission Routine*/ 

  // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 

  if (CAN_SEND_FLG1){
    CAN_SEND_FLG1=0; // reset
    packBMUmsg(&sendMessage);
    twai_transmit(&sendMessage, pdMS_TO_TICKS(1));
  } 

  if(CAN_SEND_FLG2 && CHARGING_FLAG){
    CAN_SEND_FLG2=0; // reset
    packOBCmsg(&sendMessage, OVERDIV_FLAG );
    twai_transmit(&sendMessage, pdMS_TO_TICKS(1));
  }

  // if( millis()-reference_time >= (SYNC_TIME/2)) 
  // {
  //   packBMUmsg(&sendMessage);
  //   twai_transmit(&sendMessage, pdMS_TO_TICKS(1));
  //   reference_time = millis();
  // }
  // // BCU => OBC       (500ms cycle) , only when CHARGING_FLAG = TRUE
  // if(millis()-reference_time2 >= OBC_SYNC_TIME && CHARGING_FLAG ) 
  // {
  //   packOBCmsg(&sendMessage, OVERDIV_cmd );
  //   twai_transmit(&sendMessage, pdMS_TO_TICKS(1));
  //   reference_time2 = millis();
  // }

  /* Debug and troubleshoot TWAI bus
  //Error Alert message
  uint32_t alerts_triggered;
  twai_status_info_t status_info;
  // Check if alert triggered
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
  twai_get_status_info(&status_info);
  */
    
  /*------------------------------------------Message Reception Routine*/
  
  // if RX buffer isn't cleared after its full within 1ms it alert will fired
  if (twai_receive(&receivedMessage, pdMS_TO_TICKS(1)) == ESP_OK) 
  {
    // Basic Debug
    Serial.printf("%X\n", receivedMessage.identifier);
    for (int i = 0; i < receivedMessage.data_length_code; i++) 
        Serial.printf("%X",receivedMessage.data[i]);
     Serial.println();

    checkModuleTimeout();

    // Pack data into BMS_ROSPackage: 
      // ----Driving
      unpackBMUmsg(&receivedMessage, &BMS_ROSPackage); // 200ms cycle & 1000ms cycle of faultcode
      // ----Charging
      if(CHARGING_FLAG)
        unpackOBCmsg(&receivedMessage); // 500ms cycle

    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLAG = false;
    communication_timer1 = millis();
      
  } 
  else if (millis()-communication_timer1 >= TIMEOUT_TIME)
  {
    CAN_TIMEOUT_FLAG = true;
  }
  
  /* ====================================================Task 1 : Check Fault, Shutdown==================================================== */ 
    //  Read SHUTDOWN_OK_SIGNAL signal according to the actual physical voltage of SDC
    // (digitalRead(SDCIN)) ? (SDCstat.SHUTDOWN_OK_SIGNAL = 1) : (SDCstat.SHUTDOWN_OK_SIGNAL = 0);
    // Read IMD_Ok and BSPD_OK status from shutdown circuit
    (digitalRead(IMDIN)) ? (IMD_OK = 1) : (IMD_OK = 0);
    (digitalRead(BSPDIN)) ? (BSPD_OK = 1) : (BSPD_OK = 0);
    // Confirm if the Charger is actually plugged (May change to interrupt)
    (digitalRead(OBCIN)) ? (CHARGING_FLAG = true) : (CHARGING_FLAG = false);

    // case 0 : communication timeout, via wiring or protocol error , lock MCU in this condition loop
    if( CAN_TIMEOUT_FLAG ) {
      // Reset the entired structure
      for(short i =0 ; i <BMU_NUM ; i++)
        BMS_ROSPackage[i] = BMSdata(); 
      digitalWrite(BMSOUT,LOW);
      if( millis()-shutdown_timer1 >= 400){
        Serial.println("CANBUS_INACTIVE");
        shutdown_timer1 = millis();
      }
    }
    
    // case 1: BMS detect fault , BMS_OK output LOW , while SHUTDOWN_OK_SIGNAL either read HIGH or LOW.
    else if ( !BMS_OK ) {
      digitalWrite(BMSOUT,LOW); 
    }
    // case 3 , Both BMS and SDC operate Normally , BMS output HIGH & SHUTDOWN_OK_SIGNAL read HIGH
    else if( (BMS_OK) && (SDC_OK_SIGNAL) ) {
      digitalWrite(BMSOUT,HIGH);
      // Reset status as Normal Operation
      BMS_OK = 1;
      SDC_OK_SIGNAL = 1;
    } 
    // May be this case isn't needed
    // case 3: Fault due to other system , BMS_OK HIGH , SHUTDOWN_OK_SIGNAL read LOW , IMD , and BSPD may read HIGH or low
    else {
      digitalWrite(BMSOUT,LOW); 
    /* เมื่อกี้ ปิดตัวบนแล้วทำไม work  ต้องตรวจสอบดู*/
    }

    // when fault code detected, yes this works , but when receiving with cell data msg, why sometimes the light blink?
    // Problem , once the shutdown state is reached,  and back to normal operation , the state of Relay won't reset
    

  /* ====================================================Task 2 : Determine BMS_OK relay state ==================================================== */
    (OBCstatusbit > 0) ? (BMS_OK = 0): (BMS_OK = 1);
    // Faulting Matrix , looping to check : (May need to improve to a more time efficient method)
    bool FAULTCHECKSUM;
    // IF ANY of the Module is at Critical Raise a Fault flag, BMS_OK = 0;
    FAULTCHECKSUM = BMS_ROSPackage[0].BMU_FAULTY | BMS_ROSPackage[1].BMU_FAULTY | BMS_ROSPackage[2].BMU_FAULTY |
                    BMS_ROSPackage[3].BMU_FAULTY | BMS_ROSPackage[4].BMU_FAULTY | BMS_ROSPackage[5].BMU_FAULTY |
                    BMS_ROSPackage[6].BMU_FAULTY | BMS_ROSPackage[7].BMU_FAULTY;
    (FAULTCHECKSUM > 0) ? (BMS_OK = 0) : (BMS_OK = 1);

    // IF ANY of the Module is at Warning Raise a Warn Flag , BMS_Ok , still 1 
    // Dealing with warning flag

    /*------- Coordiante BMU cell Balancing with OBC ------------*/

      bool OVERDIV_cmd = 0;
      // CMD_BMUBalanceMode();
      // Use  to Balance_DischargeNum to debug for which cells should be displayed as warning or critical
      // Serial.println(BMS_ROSPackage[0].BalancingDischarge_Cells,BIN); // Just Simple Serial debug for only first module



  
  /*-------------- BMSROS_Package debug with SD card logger -------------*/
  // Packing data to
  
  /*-------------- CORE 0 Tasks -------------*/
  // ROS , Ethernet publishy function

  /*-------------- CORE 1 Tasks -------------*/
  

  // After publishing to ROS , reset all BMU data with for loop
} 

/*******************************************************************
  ====================Local Functions Definition====================
********************************************************************/

void packBMUmsg ( twai_message_t *BCUsent) {

  BCUsent->identifier  = 0x10E0E500; // BCU ID
  BCUsent->data_length_code = 8;
  
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
void unpackBMUmsg ( twai_message_t* BCUreceived , BMSdata *BMS_ROSPackage) {
  
  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, (BCUreceived->identifier));
  // Do not monitor any messsage that is not associate with BMS channel
  if(decodedCANID.BASE_ID != 0x0E && decodedCANID.DEST != 0xE5)
    return;
  
  // Distingush SRC Address (Module ID)
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
  if(decodedCANID.PRIORITY == 0x11)
  {
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
  else if(decodedCANID.PRIORITY == 0x10)
  {
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
    FAULT_CODE_Check(i,BMS_ROSPackage);
  }
}

void checkModuleTimeout(){
  CANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID,receivedMessage.identifier);
  // Naming Convention of src address is BMU number
  // if the ID is zero meaning that it isn't here

  // Do not monitor any messsage that is not associate with BMS channel
  if(decodedCANID.BASE_ID != 0x0E && decodedCANID.DEST != 0xE5)
    return;

  if(receivedMessage.identifier == 0x00){

  }
    // else if until one condition is true


  // Set BMSdata.BMUactive = 0; for any that doesn't pass the filter condition
  // return;

}

// also for the case of Diff Voltage, the div voltage should not trigger The OK condition
void packOBCmsg ( twai_message_t *BCUsent, bool OVERDIV_CRIT_cmd ) {

  /* Set up BMS CAN frame*/
  BCUsent->identifier  = 0x1806E5F4; // refers to specification sheet
  BCUsent->data_length_code = 8;
  // Reserved
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;

  // Condition 1 iF BMS_OK OR Manual cmd DivVoltage commands,
  if(BMS_OK || OVERDIV_CRIT_cmd) {
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
void unpackOBCmsg ( twai_message_t *BCUreceived ) {
  // if message ID isnt 0x18FF50E5 , return
  if(BCUreceived->identifier != 0x18FF50E5)
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


/* ==================================Serial Debugger==============================*/
// Use Non-Block string , there's literally no perfect way to Serial Debug asynchronusly

void debugBMUmsg(int Module){

    Serial.print("BMU Operation Status: "); Serial.println(BMS_ROSPackage[Module].BMUOperationStatus);
    Serial.print("Cell balancing discharge: "); Serial.println(BMS_ROSPackage[Module].BalancingDischarge_Cells);
    Serial.print("V_CELL C1-10: ");
    // can change to vector , for easy looping funcion
    for(short i=0; i< CELL_NUM; i++){
      Serial.print(BMS_ROSPackage[Module].V_CELL[i]); Serial.print("V.  ");
    } Serial.println();
    Serial.print("V_MODULE: "); Serial.print(BMS_ROSPackage[Module].V_MODULE); Serial.println("V.  ");
    Serial.print("AVG_CELL_VOLTAGE_DIFF: ") ; Serial.print(BMS_ROSPackage[Module].DV); Serial.println("V.  ");

    Serial.print("TEMP_SENSOR T1-T2: ");
    for(short i=0; i< TEMP_SENSOR_NUM; i++){
      Serial.print(BMS_ROSPackage[Module].TEMP_SENSE[i]); Serial.print("C.  ");
    } Serial.println();

    Serial.println();

}
void debugBMUFault(int Module){
  if(!BMS_OK){
    Serial.print("OVERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERVOLTAGE_CRITICAL,HEX);
    Serial.print("UNDERVOLTAGE_CRITICAL_CELLS (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].LOWVOLTAGE_CRITICAL,HEX);
    Serial.print("OVERTEMP_CRITICAL (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERTEMP_CRITICAL, HEX);
    Serial.print("OVERDIV_CRITICAL (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERDIV_VOLTAGE_CRITICAL,HEX);
    Serial.println();
  } else {
    
  }
    Serial.print("OVERVOLTAGE_WARNING_CELLS (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERVOLTAGE_WARNING,HEX);
    Serial.print("UNDERVOLTAGE_WARNING_CELLS (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].LOWVOLTAGE_WARNING,HEX);
    Serial.print("OVERTEMP_WARNING (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERTEMP_WARNING, HEX);
    Serial.print("OVERDIV_WARNING (C1-C10): ");
    Serial.println(BMS_ROSPackage[Module].OVERDIV_VOLTAGE_WARNING,HEX);
    Serial.println();  
    
}
void debugOBCmsg(){
    
    Serial.print("Voltage from OBC: "); Serial.print(OBCVolt); Serial.println("V");
    Serial.print("Current from OBC: "); Serial.print(OBCAmp); Serial.println("A");
    Serial.print("OBC status bit"); Serial.println(OBCstatusbit);

    // Intepret Individual bit meaning
    bool *obcstatbitarray =  toBitarrayLSB(OBCstatusbit); // Status Byte
    
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
void debugSDC(){
  
  // Signal = 1 means at fault
  if(CAN_TIMEOUT_FLAG)
    Serial.println();

  // Signal = 0 means at fault
  if(!SDCstat.SHUTDOWN_OK_SIGNAL)
    Serial.println("SDC_OK_SiGNAL: OFF");
  if(!SDCstat.BMS_OK)
    Serial.println("BMS_OK = 0");
  if(!SDCstat.IMD_OK)
    Serial.println("IMD_OK = 0");
  if(!SDCstat.BSPD_OK)
    Serial.println("BSPD_OK = 0");

  
}

// Record to SD card
// Yuil sketch function to publish ROS topics

// Sub-function

void FAULT_CODE_Check(int Module , BMSdata *BMS_ROSPackage) {
  // Compile the warning flag , to signal GUI that specific module at CRITICAL
  BMS_ROSPackage[Module].BMU_FAULTY = BMS_ROSPackage[Module].OVERVOLTAGE_CRITICAL | BMS_ROSPackage[Module].LOWVOLTAGE_CRITICAL 
                                | BMS_ROSPackage[Module]. OVERTEMP_CRITICAL;
  // Compile the warning flag , to signal GUI that specific module at WARNING
  BMS_ROSPackage[Module].BMU_WARNING = BMS_ROSPackage[Module].OVERVOLTAGE_WARNING | BMS_ROSPackage[Module].LOWVOLTAGE_WARNING 
                                | BMS_ROSPackage[Module]. OVERTEMP_WARNING;
}

void CMD_BMUBalanceMode(int Module , BMSdata *BMS_ROSPackage) {
  return;
  // to reduce to just one loop , because it is inevitable , unless I use the bit shift command for each i , hmmm interesting 
  
    // OVER_DIV_VOLTAGE during charging --
      // Full condition , check for Module 1- Module BMU_NUM ,, using simple bit shift operation
      // 1. Chop up BMU operation status bit to 8, takes 4
      // bool *bitarrayholder = nullptr;
      
      // for(short i =0 ; i <BMU_NUM; i++){
      //   BMS_ROSPackage;
      //   toBitarrayMSB(BMS_ROSPackage[i].);
        

      //   // now the case that VmaxCell >= VmaxCell * 0.9
      // }

      // 4.1 Charging Ready , if so Activiate charger

      // 4.2 Low Voltage warning , if so turn on BMS low Voltage warning LED light
      // 4.3 Voltage Full (Over Voltage warning) , if so turn on the 


      // Extra** (Not in the operation status, but) OverVoltage critical , shutdown , and also turn off the charger
      // for(short i =0 ; i <BMU_NUM; i++){

      //   if(BMS_ROSPackage[i].BMUOperationStatus < VmaxCell *0.9);
      //     break;
        

      //   // now the case that VmaxCell >= VmaxCell * 0.9
      // }
      
      // BMS_ROSPackage[0].OVERDIV_VOLTAGE_WARNING; // >0.9*Vmax
      // BMS_ROSPackage[0].OVERDIV_VOLTAGE_CRITICAL; // >= 0.95 Vmax
      // // Analyze which cell is in the state of warning or critical state
      // //case 1 : WARNING => Trigger balance discharge on each specific
      // //case 2 : CRITICAL => Trigger the same mechanism , plus add extra command , to OBC to stop charging operation


}






// void unpackBMUmsg ( twai_message_t* BCUreceived) {
  
//   CANIDDecoded decodedCANID;
//   decodeExtendedCANID(&decodedCANID, (BCUreceived->identifier));
//   // Do not monitor any messsage that is not associate with BMS channel
//   if(decodedCANID.BASE_ID != 0x0E && decodedCANID.DEST != 0xE5)
//     return;
  
//   // Distingush SRC Address (Module ID)
//   byte i;
//   switch (decodedCANID.SRC)
//   {
//   case 0x01: i = 0; break;
//   case 0x02: i = 1; break;
//   case 0x03: i = 2; break;
//   case 0x04: i = 3; break;
//   case 0x05: i = 4; break;
//   case 0x06: i = 5; break;
//   case 0x07: i = 6; break;
//   case 0x08: i = 7; break;
//   }


//   /*  Message Priority 11 :: BMUModule & Cells data  */
//   if(decodedCANID.PRIORITY == 0x11){
//     // Sort out the cell message
//     switch (decodedCANID.MSG_NUM) 
//     { 
//       // MSG1 == Operation status
//       case 1:
//         // Operation status
//         BMS_ROSPackage[i].BMUOperationStatus = BCUreceived->data[0];
//         // Balancing Discharge cell number
//         BMS_ROSPackage[i].BalancingDischarge_Cells = mergeHLbyte(BCUreceived->data[1],BCUreceived->data[2]);
//         // Vbatt (Module) , dVmax(cell)
//         BMS_ROSPackage[i].V_MODULE = BCUreceived->data[3]; 
//         BMS_ROSPackage[i].DV = BCUreceived->data[4]; 
//         // Temperature sensor
//         BMS_ROSPackage[i].TEMP_SENSE[0] = BCUreceived->data[5];
//         BMS_ROSPackage[i].TEMP_SENSE[1] = BCUreceived->data[6];
        
//         break;

//       case 2:
//         // Low series Side Cell C1-C7
//         for(short j=0; j< 8; j++)
//           BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i]*0.02;

//         break;

//       case 3:
//         // High series side Cell C8-C10
//         for( short j = 8; j <= 15; j++ )
//           BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i-8]*0.02;
//         break;
//     }
//   }

//   /*  Message Priority 10 :: FaultCode  */
//   /*   Once Critical state is reached , warning stop , and the Car Shutdown Immediately   */
//   else if(decodedCANID.PRIORITY == 0x10){
//     // Aggregate data to ROS topics , display which specific cell is at fault
//     switch (decodedCANID.MSG_NUM)
//     {
//       case 1:

//         // Merge H and L byte of each FaultCode back to 10 bit binary
//         BMS_ROSPackage[i].OVERVOLTAGE_WARNING =  mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
//         BMS_ROSPackage[i].OVERVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] );  
//         BMS_ROSPackage[i].LOWVOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] );
//         BMS_ROSPackage[i].LOWVOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ); 
//         break;
//       case 2:
       
//         // Merge H and L byte of each FaultCode back to 10 bit binary
//         BMS_ROSPackage[i].OVERTEMP_WARNING = mergeHLbyte( BCUreceived->data[0], BCUreceived->data[1] );  
//         BMS_ROSPackage[i].OVERTEMP_CRITICAL = mergeHLbyte( BCUreceived->data[2], BCUreceived->data[3] ); 
//         BMS_ROSPackage[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte( BCUreceived->data[4], BCUreceived->data[5] ); 
//         BMS_ROSPackage[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] );
//         break;
//     }
    
//   }
// }

