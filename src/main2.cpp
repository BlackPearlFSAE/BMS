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
// #define BMSOUT LED_BUILTIN - SOC_GPIO_PIN_COUNT
// #define BMSOUT LED_BUILTIN  // OUTPUT Fault Signal to BMS relay
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
#define OBC_ADD 0x1806E5F4
#define BCU_ADD 0x01EE5000

/**************** Setup Variables *******************/
twai_message_t sendMessage;
twai_message_t receivedMessage;

// Software Timer reference
unsigned long reference_time = 0; unsigned long reference_time2 = 0;
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer1 = 0;
// Ticker , Timer interrupt
// Ticker ticker1; // Debugging BMU Cell msg
// Ticker ticker2; // Debugging BMU Fault code msg
// Ticker ticker3; // Debugging OBC Fault code msg
// Ticker ticker4; // Debugging OBC Fault code msg
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// Data Aggregation , and Relay to Other Sub System , e.g. Telemetry , DataLogger, BMS GUI
// Report BMS data, and Faulty status code // Serialize this data in 8 bit buffer

/* Data send to Remote System */
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


    bool BMUconnected = 1;   // Default as Active true , means each BMU is on the bus
    
    // Status
    uint16_t BalancingDischarge_Cells = 0;
    uint8_t BMUreadytoCharge = 0;
    
    // Reset call
    BMSdata() {
          memset(V_CELL, 0, sizeof(V_CELL));
          memset(TEMP_SENSE, 0, sizeof(TEMP_SENSE));
          } 
  }; 

  
  // struct OBCdata {
  //   uint8_t OBCVolt = 0;
  //   uint8_t OBCAmp = 0;
  //   uint8_t OBCstatusbit = 0 ;   // During Charging
  // };
  // Physical condition of OBC On board charger , charging power , and safety information
  uint8_t OBCVolt = 0;
  uint8_t OBCAmp = 0;
  uint8_t OBCstatusbit = 0 ;   // During Charging

  // Physical condition of SDC and LV Circuit, safety information , and relay it to Telemetry system
  struct SDCstatus {
    bool SHUTDOWN_OK_SIGNAL = 1; // AIR+
    bool BMS_OK = 1; // AMS_OUT+ ,AMS_GND
    bool IMD_OK = 1; // IMD_OUT
    bool BSPD_OK = 1; // BSPD_OUT
  };


// ACCUMULATOR Data , Local to BCU (Make this a struct later , or not? , I don't want over access)
bool  ACCUMULATOR_FULL;
bool  ACCUMULATOR_LOW_VOLTAGE;
float ACCUMULATOR_VOLTAGE = 0.0; 
float ACCUM_MAXVOLTAGE = VMAX_CELL * CELL_NUM; // Expected
float ACCUM_MINVOLTAGE = VMIN_CELL * CELL_NUM; // Expected

// Default Parameter
byte  VmaxCell ,VminCell ,TempMaxCell ,dVmax;

// Flag
const bool EEPROM_WRITE_FLAG = 0;

bool ACCUM_OK = 1;
bool ACCUM_CHG_READY_OK = 0;
bool ACCUM_OVERDIV_OK = 0;
bool CHARGER_PLUGGED = false; // (use basic digitalRead())
bool CAN_TIMEOUT_FLG = false;
bool OBC_OK = 1;
volatile bool CAN_SEND_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;


BMSdata BMS_ROSPackage[BMU_NUM];
SDCstatus SDCstat;
// OBCdata OBC_ROSPackage;

/**************** Local Function Delcaration *******************/

void packBMUmsg ( twai_message_t *BCUsent, uint16_t Sync_time,  bool &is_charger_plugged);
void packOBCmsg ( twai_message_t *BCUsent, bool &BMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full);
void unpackOBCmsg ( twai_message_t *BCUreceived );
void unpackBMUmsg ( twai_message_t *BCUreceived, BMSdata *BMS_ROSPackage);
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();
void debugSDC();
void checkModuleTimeout();
void checkACCUMULATORvoltage ();

void immediateCheck_ACCUM_OVERDIV_OK(uint16_t OVERDIV_CRITICALstatusbit);
void immediateCheck_BMU_FAULTY(int Module , BMSdata *BMS_ROSPackage);
void twaiTroubleshoot();

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
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == !ESP_OK) {
        Serial.println("Failed to reconfigure alerts");
        while(1);
      }
    }

  // Setup timer for 100ms intervals
  unsigned int time1 = (SYNC_TIME/2) * 1000 ;
  My_timer1 =  timerBegin(0, 80, true);  // Timer with prescaler
  timerAttachInterrupt(My_timer1, &onTimer1, true);
  timerAlarmWrite(My_timer1, time1, true);  // 100ms interval
  timerAlarmEnable(My_timer1);

  // // Setup timer for 500ms intervals 
  unsigned int time2 = (OBC_SYNC_TIME)*1000 ;
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer2, true);
  timerAlarmWrite(My_timer2, time2, true);  // 500ms interval
  timerAlarmEnable(My_timer2);

    Serial.println("BCU__initialized__");
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/

void loop(){
  /* CORE1 for coordinating BMS and Electrical System*/
  
  /* CORE0 for coordinating with Telemetry socket CAN*/
  // digitalWrite(BMSOUT, 1);

  // Later priority Logic shifter reading
    bool &AIRplus = SDCstat.SHUTDOWN_OK_SIGNAL; 
    bool &BMS_OK = SDCstat.BMS_OK;
    bool &IMD_OK = SDCstat.IMD_OK; 
    bool &BSPD_OK = SDCstat.BSPD_OK; 
    // Read SDC output signal to AIR+
    (digitalRead(SDCIN)) ? (AIRplus = 1) : (AIRplus = 0);
    // Read IMD_Ok and BSPD_OK status from SDC
    (digitalRead(IMDIN)) ? (IMD_OK = 1) : (IMD_OK = 0);
    (digitalRead(BSPDIN)) ? (BSPD_OK = 1) : (BSPD_OK = 0);
    // Confirm if the Charger is actually plugged (May change to interrupt)
    (digitalRead(OBCIN)) ? (CHARGER_PLUGGED = true) : (CHARGER_PLUGGED = false);

  /* ==================================================== Task 0 : Communication ====================================================*/
  /*------------------------------------------Message Transmission Routine*/ 
  
  // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 
  if (CAN_SEND_FLG1)
  {
    CAN_SEND_FLG1=0; // reset
    packBMUmsg(&sendMessage, SYNC_TIME , CHARGER_PLUGGED);
    twai_transmit(&sendMessage, 1);
  } 
  if(CAN_SEND_FLG2 && CHARGER_PLUGGED)
  {
    CAN_SEND_FLG2=0; // reset
    packOBCmsg(&sendMessage, BMS_OK, ACCUM_CHG_READY_OK, ACCUM_OVERDIV_OK, ACCUMULATOR_FULL);
    twai_transmit(&sendMessage, 1);
  }
 
  /*------------------------------------------Message Reception Routine*/

  // if RX buffer isn't cleared after its full within 1ms it alert will fired
  if (twai_receive(&receivedMessage, 1) == ESP_OK) 
  {
    // Set state BMS_OK = 1
    // digitalWrite();
    // Basic Debug
    // Serial.printf("%X\n", receivedMessage.identifier);
    // for (int i = 0; i < receivedMessage.data_length_code; i++) 
    //     Serial.printf("%X",receivedMessage.data[i]);
    //  Serial.println();

    checkModuleTimeout();

    // Unpacke CAN frame to BMS_ROSPackage: 
      // ----Driving
      unpackBMUmsg(&receivedMessage, &(BMS_ROSPackage[0])); // 200ms cycle & 1000ms cycle of faultcode
      // ----Charging
      if(CHARGER_PLUGGED)
        unpackOBCmsg(&receivedMessage); // 500ms cycle

    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLG = false;
    communication_timer1 = millis();
  } 
  // if no byte received from CAN bus , run this code and return until the bus is active
  else if (millis()-communication_timer1 >= TIMEOUT_TIME){
    // Pull BMSOUT = LOW
    digitalWrite(BMSOUT,LOW);
    CAN_TIMEOUT_FLG = true;
    if( millis()-shutdown_timer1 >= 400 ){
        Serial.println("NO_BYTE_RECV");
        shutdown_timer1 = millis();
    }
    // Reset data structure back to default, to prevent reading garbage value
    for (int i = 0; i < BMU_NUM; i++) {
      BMS_ROSPackage[i].~BMSdata(); // Explicitly call destructor (optional)
      new (&BMS_ROSPackage[i]) BMSdata(); // Placement new to reinitialize
    }
    SDCstat = SDCstatus();
    // digitalWrite(BMSOUT,BMS_OK);
    // Serial.println("NO_BYTE_RECV");
    return;
  }
    
  
  /* ====================================================Task 2 : Determine BMS_OK relay state ==================================================== */
    
    // Pulling data from BMS_ROSPackage to decide on BMS_OK state
    // Faulting Matrix , looping to check : (May need to improve to a more time efficient method)

    // IF ANY of the Module is at Critical Raise a Fault flag, BMS_OK = 0;
    // Check Fault Condition of all Module 


    // ACCUM_OK =  BMS_ROSPackage[0].BMU_FAULTY | BMS_ROSPackage[1].BMU_FAULTY | BMS_ROSPackage[2].BMU_FAULTY |
                // BMS_ROSPackage[3].BMU_FAULTY | BMS_ROSPackage[4].BMU_FAULTY | BMS_ROSPackage[5].BMU_FAULTY |
                // BMS_ROSPackage[6].BMU_FAULTY | BMS_ROSPackage[7].BMU_FAULTY;
    ACCUM_OK =  BMS_ROSPackage[0].BMU_FAULTY | BMS_ROSPackage[1].BMU_FAULTY;
    
    (ACCUM_OK > 0) ? (BMS_OK = 0) : (BMS_OK = 1);
    

    /*------- Coordiante BMU cell Balancing with OBC ------------*/
    // Check if BMU must be ready to charge
    // ACCUM_CHG_READY_OK =  BMS_ROSPackage[0].BMUreadytoCharge & BMS_ROSPackage[1].BMUreadytoCharge & BMS_ROSPackage[2].BMUreadytoCharge &
    //                       BMS_ROSPackage[4].BMUreadytoCharge & BMS_ROSPackage[5].BMUreadytoCharge & BMS_ROSPackage[6].BMUreadytoCharge & 
    //                       BMS_ROSPackage[7].BMUreadytoCharge;

    // ACCUM_CHG_READY_OK =  BMS_ROSPackage[0].BMUreadytoCharge & BMS_ROSPackage[1].BMUreadytoCharge ;


    // // if BMU(s) respond with Charger ready OK = 1 , then we start the operation
    // (OBCstatusbit > 0) ? (OBC_OK = 0): (OBC_OK = 1);

    // if(CHARGER_PLUGGED)
    //   ((ACCUM_OK & OBC_OK) > 0) ? (BMS_OK = 0) : (BMS_OK = 1);
  
    // // Convert V_MODULE back to physical value
    // ( ACCUMULATOR_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE) ? (ACCUMULATOR_FULL = 1) : (ACCUMULATOR_FULL = 0) ;

    // (ACCUMULATOR_VOLTAGE <= 1.12 * ACCUM_MINVOLTAGE) ? (ACCUMULATOR_LOW_VOLTAGE = 1) : (ACCUMULATOR_LOW_VOLTAGE = 0) ;

    // // case 1 Full voltage
    // (ACCUMULATOR_FULL) ? (BMS_OK = 0) : (BMS_OK = 1);
    // /* Signal to */
    // // case 2 Low Voltage
    // (ACCUMULATOR_LOW_VOLTAGE) ? (BMS_OK = 0) : (BMS_OK = 1);
    /*Signal to LowVoltage light*/

    // Serial Debug for which cells should be displayed as warning or critical
    // Serial.println(BMS_ROSPackage[0].BalancingDischarge_Cells,BIN); 

  
  /* ====================================================Task 1 : Check Fault, Shutdown ==================================================== */ 
    
    // case 0 : communication timeout, via wiring or protocol error , lock MCU in this condition loop
    // if( CAN_TIMEOUT_FLG ) {
    //   // Reset the entired structure
    //   // for(short i =0 ; i <BMU_NUM ; i++)
    //   //   BMS_ROSPackage[i] = BMSdata(); 
    //   // Timeout we set BMS_OK as 0
    //   // BMS_OK = 0; // Reaffirm BMS_OK state to be 0 , as No Byte recieve
    //   digitalWrite(BMSOUT,BMS_OK);
    //   if( millis()-shutdown_timer1 >= 400){
    //     Serial.println("NO_BYTE_RECV");
    //     shutdown_timer1 = millis();
    //   }
    //   return;
    // }
    // // case 1: BMS detect fault , BMS_OK output LOW 
    // else if ( !BMS_OK ) {
    //   digitalWrite(BMSOUT,BMS_OK); 
    // }
    // // case 2 , BMS operate Normally , BMS output HIGH
    // else if (BMS_OK) {
    //   digitalWrite(BMSOUT,BMS_OK);
    //   // Reset affirm status as Normal Operation
    //   // BMS_OK = 1;
    // }

    

    // case 1: BMS detect fault , BMS_OK output LOW 
    if ( !BMS_OK ) {
      digitalWrite(BMSOUT,LOW); 
    }
    // case 2 , BMS operate Normally , BMS output HIGH
    else if (BMS_OK) {
      digitalWrite(BMSOUT,HIGH);
      // Reset affirm status as Normal Operation
      // BMS_OK = 1;
    }  
    if(millis()-reference_time >= 400){
      Serial.println(ACCUM_OK);
      Serial.print("BMSOK");Serial.println(BMS_OK);
      reference_time = millis();
    }
    
    // Whilst happend AIR+ can be any affect by any fault relay , BSPD , IMD.

    // when fault code detected, yes this works , but when receiving with cell data msg, why sometimes the light blink?
    // Problem , once the shutdown state is reached,  and back to normal operation , the state of Relay won't reset
  
  /*-------------- BMSROS_Package debug with SD card logger -------------*/
  // Packing data to SD
  // packing data to Remote System using CORE0
  /*-------------- CORE 0 Tasks -------------*/
  // ROS , Ethernet publishy function

  /*-------------- CORE 1 Tasks -------------*/
  // After publishing to ROS , reset all BMU data with for loop

  // reset state with default constructor at the end of all task 
  
} 

/*******************************************************************
  ====================Local Functions Definition====================
********************************************************************/

void packBMUmsg ( twai_message_t *BCUsent, uint16_t Sync_time,  bool &is_charger_plugged) {

  BCUsent->identifier  = BCU_ADD; // BCU ID
  BCUsent->data_length_code = 8;
  // BMU synchronize time
  BCUsent->data[0] = Sync_time;
  // Notify Charge
  (is_charger_plugged) ? (BCUsent->data[1] = 1) : (BCUsent->data[1] = 0);
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
    // Sort out the cell message number
    switch (decodedCANID.MSG_NUM) 
    { 
      // MSG1 == Operation status
      case 1:
        // Charging Ready
        BMS_ROSPackage[i].BMUreadytoCharge = BCUreceived->data[0];
        // Balancing Discharge cell number
        BMS_ROSPackage[i].BalancingDischarge_Cells = mergeHLbyte(BCUreceived->data[1],BCUreceived->data[2]);
        // Vbatt (Module) , dVmax(cell)
        BMS_ROSPackage[i].V_MODULE = BCUreceived->data[3]; 
        ACCUMULATOR_VOLTAGE += BCUreceived->data[3] * 0.2; // add to ACCUM
        BMS_ROSPackage[i].DV = BCUreceived->data[4]; 
        // Temperature sensor
        BMS_ROSPackage[i].TEMP_SENSE[0] = BCUreceived->data[5];
        BMS_ROSPackage[i].TEMP_SENSE[1] = BCUreceived->data[6];
        break;

      case 2:
        // Low series Side Cell C1-C8
        for(short j=0; j< 8; j++)
          BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i];
        break;

      case 3:
        // High series side Cell C8-CellNumber
        for( short j = 8; j <= 15; j++ )
          BMS_ROSPackage[i].V_CELL[j] = BCUreceived->data[i-8];
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
        // Immediately check the first Module that contains cells with dV over critical
        immediateCheck_ACCUM_OVERDIV_OK(mergeHLbyte( BCUreceived->data[6], BCUreceived->data[7] ) ); // Figure out when should OBC hold off its operation whilist balancing
        break;
    }
    // Immediately check this Module if Faultcode pass filter condition
    immediateCheck_BMU_FAULTY(i,BMS_ROSPackage); 
    
  }
}
void packOBCmsg ( twai_message_t *BCUsent, bool &BMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full) {

  /* Set up BMS CAN frame*/
  BCUsent->identifier  = OBC_ADD; // refers to specification sheet
  BCUsent->data_length_code = 8;
  // Reserved
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;
  // Condition 1 iF BMS_OK AND ACCUM is Ready Any Module not Critically OVERDIV, ACCUMULATOR VOLTAGE ISN"T FULL YET
  if((BMS_OK && ReadytoCharge) || !OverDivCritical_Yes || !Voltage_is_Full) {
    BCUsent->data[0] = 0x0C; // V highbyte 
    BCUsent->data[1] = 0x80; // V lowbyte 72.0 V fake data -> Range 69-72-74 V
    BCUsent->data[2] = 0x00; // A Highbyte
    BCUsent->data[3] = 0x32; // A Lowbyte 5.0 A fake data
    BCUsent->data[4] = 0x00; // Control Byte 0 charger operate
  } else {
    // Condition 0 Shutdown message
    BCUsent->data[0] = 0x00; 
    BCUsent->data[1] = 0x00; 
    BCUsent->data[2] = 0x00; 
    BCUsent->data[3] = 0x00;
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
void checkModuleTimeout(){
  // Relying on the immediate reception frame isn't reliable
  // I need reception frame to accumulate for about the same time as timeout 4s
  // during that time also check for BMSROS , access its active

  // CANIDDecoded decodedCANID;
  // decodeExtendedCANID(&decodedCANID,receivedMessage.identifier);
  // // Naming Convention of src address is BMU number
  // // if the ID is zero meaning that it isn't here

  // // Do not monitor any messsage that is not associate with BMS channel
  // if(decodedCANID.BASE_ID != 0x0E && decodedCANID.DEST != 0xE5)
  //   return;

  // if(receivedMessage.identifier == 0x00){
  //   BMS_ROSPackage->BMUActive = 0;
  // }
    // else if until one condition is true

  // Set BMSdata.BMUactive = 0; for any that doesn't pass the filter condition
  return;

}
void checkACCUMULATORvoltage (){
  // Perform Immediate Check on main Loop
  // Convert V_MODULE back to physical value
  if( ACCUMULATOR_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE)
    ACCUMULATOR_FULL = 1;
  else
    ACCUMULATOR_FULL = 0;

  if(ACCUMULATOR_VOLTAGE <= 1.12 * ACCUM_MINVOLTAGE)   
    ACCUMULATOR_LOW_VOLTAGE = 1;
  else
    ACCUMULATOR_LOW_VOLTAGE = 0;
}
/* ==================================Serial Debugger==============================*/
// Use Non-Block string , there's literally no perfect way to Serial Debug asynchronusly
// maybe use this on Core 0
void debugBMUmsg(int Module){

    Serial.print("BMU Operation Status: "); Serial.println(BMS_ROSPackage[Module].BMUreadytoCharge);
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
  if(!SDCstat.BMS_OK){
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
  if(CAN_TIMEOUT_FLG)
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

// ---------------------------------------------------------------Sub-function
// Perform immediate Check during the execution of unpack();
void immediateCheck_BMU_FAULTY(int Module , BMSdata *BMS_ROSPackage) {
  // Compile the warning flag , to signal GUI that specific module at CRITICAL
  BMS_ROSPackage[Module].BMU_FAULTY = BMS_ROSPackage[Module].OVERVOLTAGE_CRITICAL | BMS_ROSPackage[Module].LOWVOLTAGE_CRITICAL 
                                | BMS_ROSPackage[Module]. OVERTEMP_CRITICAL;
  // Compile the warning flag , to signal GUI that specific module at WARNING
  BMS_ROSPackage[Module].BMU_WARNING = BMS_ROSPackage[Module].OVERVOLTAGE_WARNING | BMS_ROSPackage[Module].LOWVOLTAGE_WARNING 
                                | BMS_ROSPackage[Module]. OVERTEMP_WARNING;
}
void immediateCheck_ACCUM_OVERDIV_OK(uint16_t OVERDIV_CRITICALstatusbit) {
  // Perform Immediate Check on BMU frame
  // to reduce to just one loop , because it is inevitable , unless I use the bit shift command for each i , hmmm interesting 

      // Extra** (Not in the operation status, but) OverVoltage critical , shutdown , and also turn off the charger
      // for(short i =0 ; i <BMU_NUM; i++){

      // OVER_DIV_VOLTAGE critical 0.95* dVmax during charging --
      // Full condition , check for Module 1- Module BMU_NUM ,, using simple bit shift operation
      // Copy BMU OVERDIV Faultcode into temp variable


      // int bitHolder1 = BMS_ROSPackage[Module].OVERDIV_VOLTAGE_WARNING; // This Num must be The BMSROS.
      // int bitHolder2 = BMS_ROSPackage[Module].OVERDIV_VOLTAGE_CRITICAL; // This Num must be The BMSROS.
      unsigned int bitHolder = OVERDIV_CRITICALstatusbit;
      // First cell to be found as overdiv critical will shutdown charger (Not Shutdown the BMS_OK)
      for (int i = CELL_NUM - 1; i >= 0; i--) {
        if( (bitHolder & 1) == 1 ) {
          ACCUM_OVERDIV_OK = 1;
          break; 
        } else {
          ACCUM_OVERDIV_OK = 0;
        }
        bitHolder >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
      }

      // //case 1 : WARNING => Trigger balance discharge on each specific (BMU side)
      // //case 2 : CRITICAL => Trigger the same mechanism , plus add extra command , to OBC to stop charging operation

}
void twaiTroubleshoot(){
  //Debug and troubleshoot TWAI bus

  //Error Alert message
  uint32_t alerts_triggered;
  twai_status_info_t status_info;
  // Check if alert triggered
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1));
  twai_get_status_info(&status_info);
  Serial.println(alerts_triggered);
  
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

