// Production 17 Function DCC Decoder 
// Version 5.4  Geoff Bunza 2014,2015,2016

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

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

#define This_Decoder_Address 40     //ACCESSORY DECODER ADDRESS
                                    //Start of SWITCHES RANGE
uint8_t CV_DECODER_MASTER_RESET = 120;
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, This_Decoder_Address},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},
  {CV_DECODER_MASTER_RESET, 0},
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
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0 );
}
void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
}
extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State) {
  uint8_t Bit_State =  OutputAddr & 0x01;
  if ( Addr >= This_Decoder_Address && Addr < This_Decoder_Address+17)  //Controls This_Decoder_Address+16
	  digitalWrite( ledpins[Addr-This_Decoder_Address], Bit_State );
}

