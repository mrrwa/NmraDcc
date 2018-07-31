// Production 17 Function DCC Decoder    Dec_SMA12_LED_Groups.ino
// Version 6.01  Geoff Bunza 2014,2015,2016,2017,2018
// Now works with both short and long DCC Addesses

// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
int tim_delay = 500;

#define numleds  17
byte ledpins [] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
byte FPins_Assigned [12][5] = {   // This array defines the pins controlled by each function
    {3,4,5,6,7},  // F0
    {3,4,5,6,7},  // F1
    {3,4,5,6,7},  // F2
    {3,4,5,6,7},  // F3
    {8,9,10,11,12},  // F4
    {8,9,10,11,12},  // F5
    {8,9,10,11,12},  // F6
    {8,9,10,11,12},  // F7
    {13,14,15,16,17},  // F8
    {13,14,15,16,17},  // F9
    {13,14,15,16,17},  // F10
    {13,14,15,16,17}   // F11
 };
byte Function_Lites [12][5] = {  // This array defines the Lights/LEDs controlled in each light group per function
    {1,1,0,0,0},  // F0 Hp0
    {0,0,1,0,0},  // F1 Hp1
    {0,0,1,0,1},  // F2 Hp2
    {0,0,1,1,0},  // F3 Sh1
    {1,1,0,0,0},  // F4 Hp0
    {0,0,1,0,0},  // F5 Hp1
    {0,0,1,0,1},  // F6 Hp2
    {0,0,1,1,0},  // F7 Sh1
    {1,1,0,0,0},  // F8 Hp0
    {0,0,1,0,0},  // F9 Hp1
    {0,0,1,0,1},  // F10 Hp2
    {0,0,1,1,0}   // F11 Sh1
  };
boolean Last_Function_State[] = {false,false,false,false,false,false,false,false,false,false,false,false};

const int FunctionPin0 = 3;
const int FunctionPin1 = 4;
const int FunctionPin2 = 5;
const int FunctionPin3 = 6;
const int FunctionPin4 = 7;

const int FunctionPin5 = 8;
const int FunctionPin6 = 9;
const int FunctionPin7 = 10;
const int FunctionPin8 = 11;

const int FunctionPin9 = 12;
const int FunctionPin10 = 13;
const int FunctionPin11 = 14;     //A0
const int FunctionPin12 = 15;     //A1

const int FunctionPin13 = 16;     //A2
const int FunctionPin14 = 17;     //A3
const int FunctionPin15 = 18;     //A4
const int FunctionPin16 = 19;     //A5
NmraDcc  Dcc ;
DCC_MSG  Packet ;
uint8_t CV_DECODER_MASTER_RESET = 120;
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

#define This_Decoder_Address 24

CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address&0x7F },
  
  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, ((This_Decoder_Address>>8)&0x7F)+192 },
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, This_Decoder_Address&0xFF },
  
  // ONLY uncomment 1 CV_29_CONFIG line below as approprate DEFAULT IS SHORT ADDRESS
//  {CV_29_CONFIG,          0},                                           // Short Address 14 Speed Steps
  {CV_29_CONFIG, CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},   // Long  Address 28/128 Speed Steps  

  {CV_DECODER_MASTER_RESET, 0},
};

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
   // initialize the digital pins as an outputs
    for (int i=0; i< numleds; i++) {
      pinMode(ledpins[i], OUTPUT);
      digitalWrite(ledpins[i], LOW);
     }
  for (int i=0; i< numleds; i++) {
     digitalWrite(ledpins[i], HIGH);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  for (int i=0; i< numleds; i++) {
     digitalWrite(ledpins[i], LOW);
     delay (tim_delay/10);
  }
  delay( tim_delay);
    // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 601, FLAGS_MY_ADDRESS_ONLY, 0 );
  delay(800);
#if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET ) 
#endif 
     {
       for (int j=0; j < FactoryDefaultCVIndex; j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
         digitalWrite(ledpins[14], 1);
         delay (1000);
         digitalWrite(ledpins[14], 0);
     }  
}
void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
}
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
int f_index;
switch (FuncGrp)  { 
  case FN_0_4:    //Function Group 1 F0 F4 F3 F2 F1
    exec_function( 0, (FuncState & FN_BIT_00)>>4 );
    exec_function( 1, (FuncState & FN_BIT_01));
    exec_function( 2, (FuncState & FN_BIT_02)>>1);
    exec_function( 3, (FuncState & FN_BIT_03)>>2 );
    exec_function( 4, (FuncState & FN_BIT_04)>>3 );
    break;
  case FN_5_8:    //Function Group 1 S FFFF == 1 F8 F7 F6 F5  &  == 0  F12 F11 F10 F9 F8
    exec_function( 5, (FuncState & FN_BIT_05));
    exec_function( 6, (FuncState & FN_BIT_06)>>1 );
    exec_function( 7, (FuncState & FN_BIT_07)>>2 );
    exec_function( 8, (FuncState & FN_BIT_08)>>3 );
    break;  
  case FN_9_12:
    exec_function( 9, (FuncState & FN_BIT_09));
    exec_function( 10,(FuncState & FN_BIT_10)>>1 );
    exec_function( 11,(FuncState & FN_BIT_11)>>2 );
    break;
  }
}  
void exec_function (int f_index, int FuncState)  {
  #define fadedelay 5
       if ((FuncState==1) && (!Last_Function_State[f_index])) {
              for (int i=0; i<60; i++) {
                for (int j=0; j<5; j++) if (Function_Lites[f_index][j]==1) digitalWrite( FPins_Assigned[f_index][j],1);
                delay(fadedelay*i/60.0);
                for (int j=0; j<5; j++) if (Function_Lites[f_index][j]==1) digitalWrite( FPins_Assigned[f_index][j],0);
                delay(fadedelay-(fadedelay*i/60.0));
              }
            for (int j=0; j<5; j++)  digitalWrite( FPins_Assigned[f_index][j],Function_Lites[f_index][j]);
            Last_Function_State[f_index] = true;
          }
          else if ((FuncState==0) && Last_Function_State[f_index]) {
              for (int i=0; i<60; i++) {
                for (int j=0; j<5; j++) if (Function_Lites[f_index][j]==1) digitalWrite( FPins_Assigned[f_index][j],0);
                delay(fadedelay*i/60.0);
                for (int j=0; j<5; j++) if (Function_Lites[f_index][j]==1) digitalWrite( FPins_Assigned[f_index][j],1);
                delay(fadedelay-(fadedelay*i/60.0));
              }
            for (int j=0; j<5; j++)  digitalWrite( FPins_Assigned[f_index][j], 0);
            Last_Function_State[f_index] = false;
          }
}
