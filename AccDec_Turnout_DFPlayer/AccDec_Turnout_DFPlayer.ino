// AccDec_Turnout_DFPlayer
// Date: 2023.8.15, Author: 7M4MON
// This is DCC composite device that integrates a turnout decoder and a sound decoder.
// Six turnouts can be controlled by this device. 
// If you add an IO expander (MCP23017) ,  you can add up to 64 turnouts.
// The sound is controlled by DFPlayer mini.

// Based on 17 Switch Acessory DCC Decoder    AccDec_17LED_1Ftn.ino
// Version 6.01  Geoff Bunza 2014,2015,2016,2017,2018

#include <NmraDcc.h>
#include "SoftwareSerial.h"
#include <DFPlayer_Mini_Mp3.h>

int tim_delay = 500;

#define NUM_OF_TURNOUT 6
#define NUM_OF_EXP_TURNOUT 8        // MCP23017 has 16 output pins
#define NUM_OF_MP3 20
#define OFFSET_ADDR_TURNOUT 1
#define OFFSET_ADDR_EXP_TURNOUT 21
#define OFFSET_ADDR_MP3 40
#define TURNOUT_HOLD_TIME_MS 300

#define PIN_DCC_COM_IN 2
uint8_t turnout_control_pins[NUM_OF_TURNOUT][2] = 
{{3,4},{5,6},{7,8},{9,10},{11,12},{A0,A1}};
#define PIN_ONBORD_LED 13
#define PIN_DFPLAYER_TX A3
#define PIN_DFPLAYER_RX A2
#define PIN_DFPLAYER_VOL A6

#include <Wire.h>
#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;
bool mcp_connected = false;
uint8_t exp_turnout_control_pins[NUM_OF_EXP_TURNOUT][2] = 
{{0,1},{2,3},{4,5},{6,7},{8,9},{10,11},{12,13},{14,15}};

uint8_t ats_control_pins[2] = {A4,A5};          // if it did not use expanded IOs, A4, A5 controls ATS Alert signal.

SoftwareSerial mySerial(PIN_DFPLAYER_RX, PIN_DFPLAYER_TX); // RX, TX
int8_t mp3_volume = 20;

void set_mp3_volume(uint8_t volume_pin){
  int vol = analogRead(volume_pin);
  vol >>= 5; // 1023 -> 31
  mp3_volume = (uint8_t) vol;
  mp3_set_volume (mp3_volume);
  delay(10);
}

void init_pins(){
  for (uint8_t i = 0; i < NUM_OF_TURNOUT; i++){
      pinMode(turnout_control_pins[i][0], OUTPUT);
      digitalWrite(turnout_control_pins[i][0], LOW);
      pinMode(turnout_control_pins[i][1], OUTPUT);
      digitalWrite(turnout_control_pins[i][1], LOW);
  }
  pinMode(PIN_DCC_COM_IN, INPUT); 
  pinMode(PIN_ONBORD_LED, OUTPUT);
  digitalWrite(PIN_ONBORD_LED, HIGH);
  pinMode(PIN_DFPLAYER_RX, INPUT_PULLUP);
  pinMode(PIN_DFPLAYER_TX, OUTPUT);
  digitalWrite(PIN_DFPLAYER_TX, HIGH);
}

bool init_mcp(){
     Wire.begin();
     mcp_connected = mcp.begin_I2C();
     if (mcp_connected){
        for (uint8_t i = 0; i < NUM_OF_EXP_TURNOUT; i++){
          mcp.pinMode(exp_turnout_control_pins[i][0], OUTPUT);
          mcp.digitalWrite(exp_turnout_control_pins[i][0], LOW);
          mcp.pinMode(exp_turnout_control_pins[i][1], OUTPUT);
          mcp.digitalWrite(exp_turnout_control_pins[i][1], LOW);
        }
     }else{
      Wire.end();               // important!
      pinMode(A4, OUTPUT);      // Active Low
      digitalWrite(A4, HIGH);
      pinMode(A5, OUTPUT);
      digitalWrite(A5, HIGH);
     }
     Serial.print("mcp_connected :");
     Serial.println(mcp_connected);
     return mcp_connected;
}

void switch_turnout(uint8_t point_number, uint8_t direction){
  if(direction){
    digitalWrite(turnout_control_pins[point_number][0],HIGH);
  }else{
    digitalWrite(turnout_control_pins[point_number][1],HIGH);
  }
  delay(TURNOUT_HOLD_TIME_MS);
  digitalWrite(turnout_control_pins[point_number][0], LOW);
  digitalWrite(turnout_control_pins[point_number][1], LOW);
}

void switch_exp_turnout(uint8_t point_number, uint8_t direction){
  if(direction){
    mcp.digitalWrite(exp_turnout_control_pins[point_number][0],HIGH);
  }else{
    mcp.digitalWrite(exp_turnout_control_pins[point_number][1],HIGH);
  }
  delay(TURNOUT_HOLD_TIME_MS);
  mcp.digitalWrite(exp_turnout_control_pins[point_number][0], LOW);
  mcp.digitalWrite(exp_turnout_control_pins[point_number][1], LOW);
}

NmraDcc  Dcc ;
DCC_MSG  Packet ;

#define SET_CV_Address       24           // THIS ADDRESS IS FOR SETTING CV'S Like a Loco
#define Accessory_Address    41           // THIS ADDRESS IS THE START OF THE SWITCHES RANGE
                                          // WHICH WILL EXTEND FOR 16 MORE SWITCH ADDRESSES
										  // THIS CAN START ABOVE ADDRESS 256
uint8_t CV_DECODER_MASTER_RESET =   120;  // THIS IS THE CV ADDRESS OF THE FULL RESET
#define CV_To_Store_SET_CV_Address	121
#define CV_Accessory_Address CV_ACCESSORY_DECODER_ADDRESS_LSB



struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  // These two CVs define the Long Accessory Address
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, Accessory_Address&0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, (Accessory_Address>>8)&0x07},
  
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},

  // Speed Steps don't matter for this decoder
  // ONLY uncomment 1 CV_29_CONFIG line below as approprate DEFAULT IS SHORT ADDRESS
  //  {CV_29_CONFIG,          0},                                           // Short Address 14 Speed Steps
  //  {CV_29_CONFIG, CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
  //  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},   // Long  Address 28/128 Speed Steps  
  {CV_29_CONFIG,CV29_ACCESSORY_DECODER|CV29_OUTPUT_ADDRESS_MODE|CV29_F0_LOCATION}, // Accesory Decoder Short Address
  //  {CV_29_CONFIG, CV29_ACCESSORY_DECODER|CV29_OUTPUT_ADDRESS_MODE|CV29_EXT_ADDRESSING | CV29_F0_LOCATION},  // Accesory Decoder  Long  Address 

  {CV_DECODER_MASTER_RESET, 0},
  {CV_To_Store_SET_CV_Address, SET_CV_Address&0xFF },   // LSB Set CV Address
  {CV_To_Store_SET_CV_Address+1,(SET_CV_Address>>8)&0x3F },  //MSB Set CV Address
};

uint8_t FactoryDefaultCVIndex = 0;
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};
void setup()
{
  Serial.begin (115200);    // for debug
   // initialize the digital pins as an outputs
  init_pins();
  init_mcp();
  
  #if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET ) 
  #endif  
     {
       for (int j=0; j < FactoryDefaultCVIndex; j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
         digitalWrite(PIN_ONBORD_LED, 1);
         delay (1000);
         digitalWrite(PIN_ONBORD_LED, 0);
     }  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, PIN_DCC_COM_IN, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 601, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, CV_To_Store_SET_CV_Address);

  delay(500);
  mySerial.begin (9600);  // for DFplayer
  mp3_set_serial (mySerial);    //set Serial for DFPlayer-mini mp3 module 
  set_mp3_volume (PIN_DFPLAYER_VOL);
  mp3_play (1);
  digitalWrite(PIN_ONBORD_LED, LOW);
  Serial.println("Ready");
}
void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
}
extern void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
  if ( Addr >= OFFSET_ADDR_TURNOUT && Addr < OFFSET_ADDR_TURNOUT + NUM_OF_TURNOUT){
    // Move the turnout
    digitalWrite(PIN_ONBORD_LED, HIGH);
    Addr -= OFFSET_ADDR_TURNOUT;
    Serial.print("T");
    Serial.print(Addr);
    Serial.println(Direction);
    switch_turnout((uint8_t)Addr, Direction);
    digitalWrite(PIN_ONBORD_LED, LOW);
  }
  else if ( Addr >= OFFSET_ADDR_EXP_TURNOUT && Addr < OFFSET_ADDR_EXP_TURNOUT + NUM_OF_EXP_TURNOUT){
    // Move the turnout on io expander
    if (mcp_connected){
        digitalWrite(PIN_ONBORD_LED, HIGH);
        Addr -= OFFSET_ADDR_EXP_TURNOUT;
        Serial.print("E");
        Serial.print(Addr);
        Serial.println(Direction);
        switch_exp_turnout((uint8_t)Addr, Direction);
        digitalWrite(PIN_ONBORD_LED, LOW);
    }else{
        Serial.println("E_N");
        digitalWrite(PIN_ONBORD_LED, HIGH);
        // if it did not use expanded IOs, A4, A5 controls ATS Alert signal.
        Addr -= OFFSET_ADDR_EXP_TURNOUT;
        if (Addr < 2) {
          Serial.print("S");
          Serial.print(Addr);
          Serial.println(Direction);
          digitalWrite(ats_control_pins[Addr], !Direction); // Active Low
        }
        digitalWrite(PIN_ONBORD_LED, LOW);
    }
  }
  else if ( Addr >= OFFSET_ADDR_MP3 && Addr < OFFSET_ADDR_MP3 + NUM_OF_MP3){
    digitalWrite(PIN_ONBORD_LED, HIGH);
    // Play sound with DFPlayer Mini
    // get and set volume
    Addr -= OFFSET_ADDR_MP3;
    Serial.print("M");
    Serial.print(Addr);
    Serial.println(Direction);
    if (Direction){
      set_mp3_volume(PIN_DFPLAYER_VOL);
      mp3_play (Addr);
    }else{
       mp3_stop ();
    }
    digitalWrite(PIN_ONBORD_LED, LOW);
  }
}
