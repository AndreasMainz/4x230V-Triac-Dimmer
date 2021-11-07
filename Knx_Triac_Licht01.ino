// use i2c only
// Es weden alle 4 Lampen unterstützt
// L1 und L2 auf i2c 0x47
// L3 und L4 auf i2c 0x46 
// 5 Fensterkontakte möglich

#include <Wire.h>
#include "knx.h"
#include <ESP8266WebServer.h>
#include <KnxDevice.h>

#define KNX_SERIAL  Serial  // Serial2 not supported in Arduino
#define DEBUGSERIAL Serial1 // the 2nd serial port with TX only (GPIO2/D4) 115200 baud

// Define KONNEKTING Device related IDs

//  int angle = 0;
//  SCR_SetMode(1);
//  SCR_VoltageRegulation(1,0);
//  SCR_VoltageRegulation(2,0);
//  SCR_ChannelEnable(1);
//  SCR_ChannelEnable(2);


// ################################################
// ### DEBUG Configuration
// ################################################

uint16_t individualAddress = P_ADDR(5, 1, 113);

struct knx_data_in
{
  uint16_t Licht1 = G_ADDR(5, 1, 1);
  uint16_t Licht2 = G_ADDR(5, 1, 2);
  uint16_t Licht3 = G_ADDR(5, 1, 3);
  uint16_t Licht4 = G_ADDR(5, 1, 4);
};
knx_data_in Knx_In;

struct knx_data_out
{
  uint16_t L1_status = G_ADDR(5, 1, 5);
  uint16_t L2_status = G_ADDR(5, 1, 6);
  uint16_t L3_status = G_ADDR(5, 1, 7);
  uint16_t L4_status = G_ADDR(5, 1, 8);
};
knx_data_out Knx_Out;

// Definition of the Communication Objects attached to the device
KnxComObject KnxDevice::_comObjectsList[] = {
  /* Input */
  /* Suite-Index 0 */     KnxComObject(Knx_In.Licht1,    KNX_DPT_1_001, COM_OBJ_LOGIC_IN),
  /* Suite-Index 1 */     KnxComObject(Knx_In.Licht2,    KNX_DPT_1_001, COM_OBJ_LOGIC_IN),
  /* Suite-Index 2 */     KnxComObject(Knx_In.Licht3,    KNX_DPT_1_001, COM_OBJ_LOGIC_IN),
  /* Suite-Index 3 */     KnxComObject(Knx_In.Licht4,    KNX_DPT_1_001, COM_OBJ_LOGIC_IN),
  /* Suite-Index 4 */     KnxComObject(Knx_Out.L1_status,KNX_DPT_1_001, COM_OBJ_SENSOR),
  /* Suite-Index 5 */     KnxComObject(Knx_Out.L2_status,KNX_DPT_1_001, COM_OBJ_SENSOR),
  /* Suite-Index 6 */     KnxComObject(Knx_Out.L3_status,KNX_DPT_1_001, COM_OBJ_SENSOR),  
  /* Suite-Index 7 */     KnxComObject(Knx_Out.L4_status,KNX_DPT_1_001, COM_OBJ_SENSOR)
};

const byte KnxDevice::_comObjectsNb = sizeof(_comObjectsList) / sizeof(KnxComObject); // do no change this code
//Die IIC_Addr ist 0x47, oder höher, siehe Löt-Jumper PCB
uint8_t IIC_Addr_t = 0x47;
/*-----------------------------------------------------------------------------*/

// ################################################
// ###  ISR Routinen
// ################################################
void IRAM_ATTR ISR_T1()
{
  T1_Event_Rising = millis();
  if ((T1_Event_Rising - T1_Event_Falling) > 10) // Bouncing passed
  {
    attachInterrupt(Taster1, ISR_T1_RELEASE, RISING);
  }
}
void IRAM_ATTR ISR_T1_RELEASE()
{
  T1_Event_Falling = millis();
  timer1 = (T1_Event_Falling - T1_Event_Rising);
  if (timer1 > 10) // Bouncing passed
  {
    attachInterrupt(Taster1, ISR_T1, FALLING);
    if (timer1 > 600) T1_GA_long  = 1; // Serial.println(" -> laaaaaaaang. ");
    else              T1_GA_short = 1; // Serial.println(" -> kurz. ");
  }
}
void IRAM_ATTR ISR_T2()
{
  T2_Event_Rising = millis();
  if ((T2_Event_Rising - T2_Event_Falling) > 10) // Bouncing passed
  {
    attachInterrupt(Taster2, ISR_T2_RELEASE, RISING);
  }
}
void IRAM_ATTR ISR_T2_RELEASE()
{
  T2_Event_Falling = millis();
  timer1 = (T2_Event_Falling - T2_Event_Rising);
  if (timer1 > 10) // Bouncing passed
  {
    attachInterrupt(Taster2, ISR_T2, FALLING);
    L2_Event = 1;
    L2_State = 1;
  }
}

void IRAM_ATTR ISR_T3()
{
  T3_Event_Rising = millis();
  if ((T3_Event_Rising - T3_Event_Falling) > 10) // Bouncing passed
  {
    attachInterrupt(Taster3, ISR_T3_RELEASE, RISING);
  }
}
void IRAM_ATTR ISR_T3_RELEASE()
{
  T3_Event_Falling = millis();
  timer1 = (T3_Event_Falling - T3_Event_Rising);
  if (timer1 > 10) // Bouncing passed
  {
    attachInterrupt(Taster3, ISR_T3, FALLING);
    if (timer1 > 600) T3_GA_long  = 1; // Serial.println(" -> laaaaaaaang. ");
    else              T3_GA_short = 1; // Serial.println(" -> kurz. ");
  }
}
void IRAM_ATTR ISR_T4()
{
  T4_Event_Rising = millis();
  if ((T4_Event_Rising - T4_Event_Falling) > 10) // Bouncing passed
  {
    attachInterrupt(Taster4, ISR_T4_RELEASE, RISING);
  }
}
void IRAM_ATTR ISR_T4_RELEASE()
{ // keine Unterscheidung kurz /lang bei automatik toggle
  T4_Event_Falling = millis();
  timer1 = (T4_Event_Falling - T4_Event_Rising);
  if (timer1 > 10) // Bouncing passed
  {
    attachInterrupt(Taster4, ISR_T4, FALLING);
    L2_Event = 1;
    L2_State = 0;
  }
}
void IRAM_ATTR ISR_T5()
{
  T5_Event_Rising = millis();
  if ((T5_Event_Rising - T5_Event_Falling) > 10) // Bouncing passed
  {
    attachInterrupt(Kontakt, ISR_T5_RELEASE, RISING);
    T5_Event = 1;
    T5_State = 1;
  }
}
void IRAM_ATTR ISR_T5_RELEASE()
{
  T5_Event_Falling = millis();
  timer1 = (T5_Event_Falling - T5_Event_Rising);
  if (timer1 > 10) // Bouncing passed
  {
    attachInterrupt(Kontakt, ISR_T5, FALLING);
    T5_Event = 1;
    T5_State = 0;
  }
}


// ################################################
// ### KNX EVENT CALLBACK
// ################################################

void knxEvents(byte index) {
  // Empfangsobjekte Input
  Serial1.print("Objekt empfangen");
  switch (index) {
    case 0: //Licht1
      Status_Licht1_old = Status_Licht1;
      Status_Licht1 = Knx.read(index);
      if (Status_Licht1){ // Licht1 an
          IIC_Addr_t = 0x47;
          if (!Status_Licht1_old){Triac1(1); Status_Licht1_old = 1;}
          Knx.write(4,Status_Licht1); 
      }
      else
      { // Licht1 aus
          IIC_Addr_t = 0x47;
          if (Status_Licht1_old){Triac1(0); Status_Licht1_old = 0;}
          Knx.write(4,Status_Licht1); 
      }    
      break;
    case 1: // Licht2
      Status_Licht2_old = Status_Licht2;
      Status_Licht2 = Knx.read(index);
      if (Status_Licht2) // Licht2 an
      {
          IIC_Addr_t = 0x47;
          if (!Status_Licht2_old){Triac2(1); Status_Licht2_old = 1;}
          Knx.write(5,Status_Licht2); 
      }
      else
      { // Licht2 aus
          IIC_Addr_t = 0x47;
          if (Status_Licht2_old){Triac2(0); Status_Licht2_old = 0;}  
          Knx.write(5,Status_Licht2); 
      }
      break;
    case 2: // Licht3 
      Status_Licht3_old = Status_Licht3;
      Status_Licht3 = Knx.read(index);
      if (Status_Licht3){ // Licht3 an
          IIC_Addr_t = 0x46;
          if (!Status_Licht3_old){Triac1(1); Status_Licht3_old = 1;}
          Knx.write(6,Status_Licht3); 
      }
      else { // Licht3 aus
          IIC_Addr_t = 0x46;
          if (Status_Licht3_old){Triac1(0); Status_Licht3_old = 0;}  
          Knx.write(6,Status_Licht3); 
      }
      break;
    case 3: // Licht4 
      Status_Licht4_old = Status_Licht4;
      Status_Licht4 = Knx.read(index);
      if (Status_Licht4){ // Licht4 an
          IIC_Addr_t = 0x46;
          if (!Status_Licht4_old){Triac2(1); Status_Licht4_old = 1;}
          Knx.write(7,Status_Licht4); 
      }
      else{ // Licht4 aus
          IIC_Addr_t = 0x46;
          if (Status_Licht4_old){Triac2(0); Status_Licht4_old = 0;}  
          Knx.write(7,Status_Licht4); 
      }
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////////////
static void TriacSendCommand(uint8_t *Data)
{
  Wire.beginTransmission(IIC_Addr_t);
  Wire.write(Data[0]);
  Wire.write(Data[1]);
  Wire.write(Data[2]);
  Wire.endTransmission();
}
static void TriacSendSingle(uint8_t data0, uint8_t data1, uint8_t data2)
{
  Wire.beginTransmission(IIC_Addr_t);
  Wire.write(data0);
  Wire.write(data1);
  Wire.write(data2);
  Wire.endTransmission();
}
/******************************************************************************
  function:   Set working mode
  parameter:  Mode:
                  0: Switch mode
                  1: Voltage regulation mode
  Info:
******************************************************************************/
void TriacSetMode(uint8_t Mode)
{ // Schreibe ins Register 0x01, 0: an/aus, 1: Dimmen
  uint8_t ch[4] = {0x01, 0x00, 0x00, 0x00};
  ch[2] = Mode & 0x01;
  TriacSendCommand(ch);
  // Serial1.print("Triac SetMode 100");
  // delay(100);
}

/******************************************************************************
  function:   Set channel enable
  parameter:  none: Switch always both channels
  Info:
******************************************************************************/
void TriacDisable(void)
{ // Schreibt ebenfalls in Register0x02, schaltet beide Kanäle aus
  uint8_t T_disable[4] = {0x02, 0x00, 0x00, 0x00};
  TriacSendCommand(T_disable);
  // delay(100);
}
void TriacEnable(void)
{ // Schreibt ebenfalls in Register0x02, schaltet beide Kanäle an
  uint8_t T_enable[4] = {0x02, 0x00, 0x03, 0x00};
  TriacSendCommand(T_enable);
  // Serial1.print("Triac enable 203");
  // delay(100);
}
uint8_t CH_EN[4] = {0x02, 0x00, 0x00, 0x00};
void TriacChannelEnable(uint8_t Channel)
{ // Register 0x02, siehe Channel enable oben
  if (Channel == 1) {
    CH_EN[2] |= 0x01;
    TriacSendCommand(CH_EN);
  } else if (Channel == 2) {
    CH_EN[2] |= 0x02;
    TriacSendCommand(CH_EN);
  }
  // delay(100);
}


/******************************************************************************
  function:   Set Voltage Regulation
  parameter:  Channel:
                  1: Channel 1
                  2: Channel 2
            Angle:
                   0~179 Conduction angle
  Info:
******************************************************************************/
uint8_t Angle1[4] = {0x03, 0x00, 0x00, 0x00};
uint8_t Angle2[4] = {0x04, 0x00, 0x00, 0x00};
//Parameter 4 Set conduction angle 0-179
//If in mode 1, then the conduction angle greater than 90 degrees will be on
//If set to 180 the actual effect is 0
void TriacDimmer(uint8_t Channel,  uint8_t Angle)
{
  if (Channel == 1) {
    Angle1[2] = Angle % 180;
    TriacSendCommand(Angle1);
  } else if (Channel == 2) {
    Angle2[2] = Angle % 180;
    TriacSendCommand(Angle2);
  }
  // delay(100);
}

// Switch on binary
void Triac1(uint8_t On_off)
{
  if (On_off)TriacSendSingle(3, 0, 0x91);
  else       TriacSendSingle(3, 0, 1);

}

void Triac2(uint8_t On_off)
{
  if (On_off)TriacSendSingle(4, 0, 0x91);
  else       TriacSendSingle(4, 0, 1);
}

/******************************************************************************
  function:   SCR Reset
  parameter:  Delay:
                    0~255 Delay in milliseconds before reset
  Info:
    Reset all settings except baud rate and grid frequency,
******************************************************************************/
void TriacReset(void)
{ // Schreibe ins Register 0x06
  uint8_t ch[] = {0x06, 0x00, 0x00, 0x00};
  TriacSendCommand(ch);
  // Serial1.print("Triac SetMode 100");
  // delay(100);
}

void Triac_init(void) {
  TriacSetMode(0); // Switch Mode
  // delay(10);
  TriacChannelEnable(1);       // beide Kanäle an..
  TriacChannelEnable(2);

  // TriacEnable;     // beide Kanäle an..
  //TriacSetMode(0); // Switch Mode
  //TriacDimmer(1, 0); // Ch1 ausgedimmt
  //TriacDimmer(2, 0); // Ch2 ausgedimmt
  // Serial1.println("Triac loop init");
}

void TriacReinit(void){
  //Wire.begin(3, 1); //3 = Rx0 = SDA, 1 = Tx0 = SCL
  //delay(100);
  //TriacReset();
  //delay(200);
  TriacSetMode(0); // Switch Mode
  //delay(100);
  Triac2(0); //off
  Triac1(0); //off
  TriacChannelEnable(1);       // beide Kanäle an..
  TriacChannelEnable(2);
  Ruhetimer = 0;
  Serial1.println("Triac reinit! ");
}

////////////////////////////////////////////////////////
void progLed (bool state) {}; //nothing to do here

void setup() {
  #if 0
  EEPROM.begin(512);
  T_Boot_Mode = EEPROM.read(1);
  if (T_Boot_Mode != 'r')
  {
    delay(5000);
    Serial1.begin(115200);
    Serial1.println("No Run mode ");
    delay(5000);
    EEPROM.write(1, 'r');
    EEPROM.commit();
    Serial1.println("restart.");
    delay(1000);
    ESP.restart();
  }
  else
  {
    EEPROM.write(1, 'b');
    EEPROM.commit();
    Serial1.println("Run mode detected");
  }
  #endif
  WiFi.forceSleepBegin();  // WLan aus
  delay(5000); //Warten auf Supercap
  //////////////////////////////////// Switch ///////////////////////////////////////////////////
  // Init Inputs
  pinMode(Taster1, INPUT_PULLUP);
  pinMode(Taster2, INPUT_PULLUP);
  pinMode(Taster3, INPUT_PULLUP);
  pinMode(Taster4, INPUT_PULLUP);
  pinMode(Kontakt, INPUT_PULLUP);
  // Attach interrupts
  attachInterrupt(Taster1, ISR_T1, FALLING);
  attachInterrupt(Taster2, ISR_T2, FALLING);
  attachInterrupt(Taster3, ISR_T3, FALLING);
  attachInterrupt(Taster4, ISR_T4, FALLING);
  attachInterrupt(Kontakt, ISR_T5, FALLING);

  if (Knx.begin(KNX_SERIAL, individualAddress) == KNX_DEVICE_ERROR) {
    DEBUGSERIAL.println("knx init ERROR, stop here!!");
    while (1);
  }
  DEBUGSERIAL.println("Knx Init done");
  // Now serial is swiched to RX2/TX2!
  
  Wire.begin(3, 1); //3 = Rx0 = SDA, 1 = Tx0 = SCL
  Serial1.begin(115200);
  Serial1.println("I2c started");
  //////////////////////////////////////// Init Triac1 ///////////////////////////////////////////
  IIC_Addr_t = 0x47;
  TriacSetMode(0); // Switch Mode
  TriacDimmer(1, 0); // Ch1 ausgedimmt
  TriacDimmer(2, 0); // Ch2 ausgedimmt
  TriacChannelEnable(1);       // beide Kanäle an..
  TriacChannelEnable(2);
  //////////////////////////////////////// Init Triac2 ///////////////////////////////////////////
  IIC_Addr_t = 0x46;
  TriacSetMode(0); // Switch Mode
  TriacDimmer(1, 0); // Ch1 ausgedimmt
  TriacDimmer(2, 0); // Ch2 ausgedimmt
  TriacChannelEnable(1);       // beide Kanäle an..
  TriacChannelEnable(2);
  Serial1.println("Setup finished");
}

void loop() {
  Main_Clock++;
  /////////////////////////////////////////////////////////////////////////////////////
  Knx.task();
  /////////////////////////////////////////////////////////////////////////////////////
  
  ////////////////////////////////////// Abfrage eigene Schalter ///////////////////////////////
  if (T1_GA_short)
  {
    T1_GA_short = 0;
    // Serial1.println("T1");
    T1_Short_State = !T1_Short_State; // toggle state -> UM Taster Mode
  } else {
    if (T1_GA_long)
    {
      T1_GA_long = 0;
      T1_Long_State = !T1_Long_State; // toggle state -> UM Taster Mode
    }
  }
  if (T3_GA_short)
  {
    T3_GA_short = 0;
    // Serial1.println("T3");
    T3_Short_State = !T3_Short_State; // toggle state -> UM Taster Mode
  } else {
    if (T3_GA_long)
    {
      T3_GA_long = 0;
      T3_Long_State = !T3_Long_State; // toggle state -> UM Taster Mode
    }
  }
  // Taster Licht
  if (L2_Event) // Taster2 oder Taster4
  {
    L2_Event = 0;
    // Knx.write(5, L2_State);
  }

  ///////////////////////////////////// Fensterkontakt auf Pin10 ////////////////////////////
  if (T5_Event)
  {
    T5_Event = 0;
    Knx.write(4, T5_State);
  }
}
