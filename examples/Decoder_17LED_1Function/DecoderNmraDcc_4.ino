// Working 14 Function DCC Decoder  DccAckPin not needed
// Version 3.0  Geoff Bunza 2014
// Uses modified software servo Lib
//
#include <NmraDcc.h>

int tim_delay = 500;
#define numleds  17
byte ledpins [] = {0,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};

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

#define This_Decoder_Address 17

extern uint8_t Decoder_Address = This_Decoder_Address;
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},
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
  // initialize the digital pins as an outputs
    for (int i=1; i<= numleds; i++) {
      pinMode(ledpins[i], OUTPUT);
      digitalWrite(ledpins[i], LOW);
     }
  for (int i=1; i<= numleds; i++) {
     digitalWrite(ledpins[i], HIGH);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  for (int i=1; i<= numleds; i++) {
     digitalWrite(ledpins[i], LOW);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100, FLAGS_MY_ADDRESS_ONLY, 0 );
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}

extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
{
  switch(FuncGrp)
  {
    case FN_0_4:
      digitalWrite( FunctionPin0, (FuncState & FN_BIT_00) );
      digitalWrite( FunctionPin1, (FuncState & FN_BIT_01) );
      digitalWrite( FunctionPin2, (FuncState & FN_BIT_02) );
      digitalWrite( FunctionPin3, (FuncState & FN_BIT_03) );
      digitalWrite( FunctionPin4, (FuncState & FN_BIT_04) );
      break;
      
    case FN_5_8:
      digitalWrite( FunctionPin5, (FuncState & FN_BIT_05) );
      digitalWrite( FunctionPin6, (FuncState & FN_BIT_06) );
      digitalWrite( FunctionPin7, (FuncState & FN_BIT_07) );
      digitalWrite( FunctionPin8, (FuncState & FN_BIT_08) );
      break;
      
    case FN_9_12:
      digitalWrite( FunctionPin9,  (FuncState & FN_BIT_09) );
      digitalWrite( FunctionPin10, (FuncState & FN_BIT_10) );
      digitalWrite( FunctionPin11, (FuncState & FN_BIT_11) );
      digitalWrite( FunctionPin12, (FuncState & FN_BIT_12) );
      break;

    case FN_13_20:
      digitalWrite( FunctionPin13, (FuncState & FN_BIT_13) );
      digitalWrite( FunctionPin14, (FuncState & FN_BIT_14) );
      digitalWrite( FunctionPin15, (FuncState & FN_BIT_15) );
      digitalWrite( FunctionPin16, (FuncState & FN_BIT_16) );
      break;
      
    case FN_21_28:
      break;
  }
}

