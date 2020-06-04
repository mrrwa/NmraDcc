// Production 17 Function DCC Decoder   Dec_Dir_and_Fade.ino
// Version 6.01  Geoff Bunza 2014,2015,2016,2017,2018
// Now works with both short and long DCC Addesses
// LED control is dependent on direction of travel and Fade can be added

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

#include <NmraDcc.h>

int tim_delay = 500;
#define numleds  17
byte ledpins [] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};    //Defines all possible LED pins

//  IMPORTANT:
// The following list defines how each of the 17 function pins operate:
// a 0 allows for normal On/Off control with fade on and fade off
// a 1 allows for normal control when the decoder sees a forward speed setting, reverse turns the LED off
// a 2 allows for normal control when the decoder sees a reverse speed setting, forward turns the LED off
byte led_direction [] = {0,1,2,0,1,1,1,1,2,2,2,1,1,1,2,0,0};        //0=On/Off, 1=On Forward, 2=On Reverse

boolean led_last_state [] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};  //last state of led
boolean Last_Function_State[] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};  //These hold the last Fx assignments
uint8_t Decoder_direction = DCC_DIR_FWD;
uint8_t Last_Decoder_direction = 0; 
int fade_time = 170;
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
uint8_t FactoryDefaultCVIndex = 0;
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};
void setup()
{
   //Serial.begin(115200);
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
  Dcc.init( MAN_ID_DIY, 601, FLAGS_MY_ADDRESS_ONLY, 0 );
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
	exec_function( 12,(FuncState & FN_BIT_12)>>3 );
    break;

  case FN_13_20:   //Function Group 2 FuncState == F20-F13 Function Control
      exec_function( 13, (FuncState & FN_BIT_13));
      exec_function( 14, (FuncState & FN_BIT_14)>>1 );
      exec_function( 15, (FuncState & FN_BIT_15)>>2 );
      exec_function( 16, (FuncState & FN_BIT_16)>>3 );
      break;

  }
}
void exec_function (int f_index, int FuncState)  {
       if ((FuncState==1) && (!Last_Function_State[f_index])) {
            Last_Function_State[f_index] = true;
			Set_LED (f_index,true);
          }
          else if ((FuncState==0) && Last_Function_State[f_index]) { 
            Last_Function_State[f_index] = false;
			Set_LED (f_index,false);
          }
}
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION ForwardDir, DCC_SPEED_STEPS SpeedSteps )  {
  Last_Decoder_direction = Decoder_direction;
  Decoder_direction = ForwardDir;
  if ( Decoder_direction==Last_Decoder_direction) return;
  for (int i=0; i<numleds; i++) {
    if (Decoder_direction!=0 && led_direction[i]==1 && Last_Function_State[i] && led_last_state[i]==false ) Switch_LED (i);
    if (Decoder_direction!=0 && led_direction[i]==2 && Last_Function_State[i] && led_last_state[i]==true ) Switch_LED (i);
    if (Decoder_direction==0 && led_direction[i]==2 && Last_Function_State[i] && led_last_state[i]==false ) Switch_LED (i);
    if (Decoder_direction==0 && led_direction[i]==1 && Last_Function_State[i] && led_last_state[i]==true ) Switch_LED (i);    
  }
}
void Set_LED (int Function, boolean led_state) {
boolean start_state = !led_state;
boolean end_state = led_state;
    switch (led_direction[Function]) {
	  case 0:                                      //0=On/Off
      if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
	  break;
	  case 1:                                     //1=On Forward
	  if (Decoder_direction!=0) {
		  if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
	  }
	  break;
	  case 2:                                     //2=On Reverse
	  if (Decoder_direction==0)  {
      if (led_last_state[Function] == led_state) return;
      Switch_LED (Function);
	  }
	  break;
	  default:
	  break;
	}
}

void Switch_LED (int Function) {
  float time_fraction;
  int del_temp;
  boolean start_state = led_last_state[Function];
  boolean end_state = !led_last_state[Function];
  for (int loop_time=0; loop_time<fade_time; loop_time++)  {
        time_fraction = (float (loop_time))/(float (fade_time));
        digitalWrite (ledpins[Function], start_state);
        del_temp = 1000 - (1000.*time_fraction);
        if (del_temp<0) del_temp=0;
        delayMicroseconds (del_temp);
        digitalWrite (ledpins[Function], end_state);
        delayMicroseconds (1000.*time_fraction);
        }
  led_last_state[Function] = end_state;
}

