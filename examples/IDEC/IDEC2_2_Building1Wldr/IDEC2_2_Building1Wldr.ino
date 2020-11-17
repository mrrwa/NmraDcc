// Interactive Decoder Random Building Lighting DCC Decoder   IDEC2_2_Building1Wldr.ino
// Version 1.08 Geoff Bunza 2020
// Works with both short and long DCC Addesses
// This decoder will control Random Building Lighting
// F0=Master Function OFF = Function ON DISABLES the decoder
// Input Pin for Decoder Disable  Pin 3 Active LOW
/*
     F0 == Master Decoder Disable == ON
     F1 == Welder 1 Disable == ON
	 
PRO MINI PIN ASSIGNMENT:
2 - DCC Input
3 - Input Pin for MasterDecoderDisable Active LOW
4 - LED Blue Welder1
5 - LED White Welder1
6 - LED
7 - LED
8 - LED
9 - LED
10 - LED
11 - LED
12 - LED 
13 - LED
14 A0 - LED
15 A1 - LED
16 A2 - LED
17 A3 - LED
18 A4 - LED
19 A5 - LED
*/

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
#define runEvery(t) for (static typeof(t) _lasttime;\
                        (typeof(t))((typeof(t))millis() - _lasttime) > (t);\
                        _lasttime += (t))
int building_tim_delay;
int welder1_tim_delay;
byte welder1_on = 0;
int welder1_delta;
long delta = 0; 
int tctr, tctr2, i;

int numleds = 16;             // Number of Output pins to initialize
int num_active_functions = 2;  // Number of Functions stating with F0
byte fpins [] = {4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};  //These are all the Output Pins

const int MasterDecoderDisablePin = 3; // D3  Master Decoder Disable Input Pin Active LOW
const int Welder1BluePin =  4;  // Blue LED simulation Welder 1
const int Welder1WhitePin = 5;  // White LED simulation Welder 1
const int FunctionPin0 = 20;   // Input Master Pin Disable Active LOW {;aceholder
const int FunctionPin1 = 20;   // A0 LED
const int FunctionPin2 = 20;   // A1 LED
const int FunctionPin3 = 20;   // A2 LED
const int FunctionPin4 = 20;   //A3 LED
const int FunctionPin5 = 20;   //A4 LED
const int FunctionPin6 = 20;   //A5 LED
const int FunctionPin7  = 20;   // Place holders ONLY
const int FunctionPin8  = 20;   // Place holders ONLY
const int FunctionPin9  = 20;   // Place holders ONLY
const int FunctionPin10 = 20;   // Place holders ONLY
const int FunctionPin11 = 20;   // Place holders ONLY
const int FunctionPin12 = 20;   // Place holders ONLY
const int FunctionPin13 = 20;   // Place holders ONLY
const int FunctionPin14 = 20;   // Place holders ONLY
const int FunctionPin15 = 20;   // Place holders ONLY
const int FunctionPin16 = 20;   // Place holders ONLY
int MasterDecoderDisable = 0;
int MasterDisable_value = 0;
int Disable_welder1 = 0;						

NmraDcc  Dcc ;
DCC_MSG  Packet ;
uint8_t CV_DECODER_MASTER_RESET = 120;
int t;  // temp
struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};
QUEUE *ftn_queue = new QUEUE[3];

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
  {30, 0},    //  F0 Config 0=DISABLE On/Off,1=Disable Welder 1,2=Deisable Welder2
  {31, 1},    //  F1 Config 0=DISABLE On/Off,1=Disable Welder 1,2=Deisable Welder2
  {50, 90},   //  Master Building Time Delay 0-255 255=Slowest
  {51, 127},  //  Welder1 Time Constant
  {52, 0},    //  Extra
  {53, 0},    //  Extra
};

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};
// NOTE: NO PROGRAMMING ACK IS SET UP TO MAXIMAIZE 
// OUTPUT PINS FOR FUNCTIONS

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  pinMode (MasterDecoderDisablePin,INPUT_PULLUP); //  Master Decoder Disable Input Pin Active LOW
  uint8_t cv_value;
    // initialize the digital pins as outputs
	for (int i=0; i < numleds; i++) {
	  pinMode(fpins[i], OUTPUT);
	  digitalWrite(fpins[i], 0);    // All OUPUT pins initialized LOW
	}
	for (int i=0; i< numleds; i++) {   //As a test turn all ON in sequence
     digitalWrite(fpins[i], HIGH);
     delay (60);
    }
    delay(400);
    for (int i=0; i< numleds; i++) {  //Now turn all OFF in sequence
      digitalWrite(fpins[i], LOW);
      delay (60);
    }
   // Setup which External Interrupt, the Pin it's associated with that we're using 
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
         digitalWrite(fpins[10], 1);
         delay (500);
         digitalWrite(fpins[10], 0);
     }
  for ( i=0; i < num_active_functions; i++) {
    cv_value = Dcc.getCV(30+i) ;   
#ifdef DEBUG
    Serial.print(" cv_value: ");
    Serial.println(cv_value, DEC) ;
#endif
    switch ( cv_value ) {
      case 0:   // Master Decoder Disable
        MasterDecoderDisable = 0;
		    if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
        break;
      case 1:   //  F1 Disables Welder 1
          Disable_welder1 = 0;   // Initialized
        break;         
       case 2:   
         break;
	   case 3:   // NEXT FEATURE for the Future
        break;  
       default:
         break;
    }
  }
  building_tim_delay = int(Dcc.getCV(50)) * 11 ;

  welder1_tim_delay = int(Dcc.getCV(51)) * 21 ;
  
}  // end setup

// ================================================================
void loop()
{
   //MUST call the NmraDcc.process() method frequently 
   // from the Arduino loop() function for correct library operation
   Dcc.process();
   delay(1);
   
   //  INPUT OVER RIDE   // Check Master Input Over ride
   MasterDecoderDisable = 0;
   if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
     else MasterDecoderDisable = MasterDisable_value & 1;
   
   // Random Building Lights
   runEvery(building_tim_delay)  digitalWrite(fpins[random (2,numleds)], lightsw() );
   
   // Welder1
	if ((MasterDecoderDisable == 0)&&(Disable_welder1==0)) { 
	  runEvery(welder1_tim_delay)  {welder1_on = random(20,50); welder1_delta=random(23,133); }
	}
    runEvery(welder1_delta)  digitalWrite(Welder1WhitePin,run_welder1_wsw() );
	  runEvery(welder1_delta)  digitalWrite(Welder1BluePin,run_welder1_bsw() );

}  //end loop

boolean run_welder1_wsw() {
  if ((MasterDecoderDisable == 1)||(welder1_on<=0)) return LOW;  //Eventually turn all lights OFF
  welder1_on--;
  if (random(0,100)>48) return HIGH;  //48 represents a 52% ON time
    else return LOW;
}  // end run_welder1_wsw
boolean run_welder1_bsw() {
  if ((MasterDecoderDisable == 1)||(welder1_on<=0)) return LOW;  //Eventually turn all lights OFF
  welder1_on--;
      if (random(0,100)>35) return HIGH;  //35 represents a 65% ON time
    else return LOW;
}  // end run_welder1_bsw

boolean lightsw() {
  if (MasterDecoderDisable == 1) return LOW;  //Eventually turn all lights OFF
  if (random(0,100)>40) return HIGH;  //40 represents a 60% ON time
    else return LOW;
}  // end lightsw

void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
#ifdef DEBUG
   Serial.print("Addr= ");
   Serial.println(Addr, DEC) ;
   Serial.print("FuncState= ");
   Serial.println(FuncState, DEC) ;
#endif
  switch(FuncGrp)
  {
  case FN_0_4:    //Function Group 1 F0 F4 F3 F2 F1
	  exec_function( 0, FunctionPin0, (FuncState & FN_BIT_00)>>4 );
	  exec_function( 1, FunctionPin1, (FuncState & FN_BIT_01));
	  exec_function( 2, FunctionPin2, (FuncState & FN_BIT_02)>>1);
	  //exec_function( 3, FunctionPin3, (FuncState & FN_BIT_03)>>2 );
	  //exec_function( 4, FunctionPin4, (FuncState & FN_BIT_04)>>3 );
      break;
  case FN_5_8:    //Function Group 1 S FFFF == 1 F8 F7 F6 F5  &  == 0  F12 F11 F10 F9 F8
  	  //exec_function( 5, FunctionPin5, (FuncState & FN_BIT_05));
	  //exec_function( 6, FunctionPin6, (FuncState & FN_BIT_06)>>1 );
	  //exec_function( 7, FunctionPin7, (FuncState & FN_BIT_07)>>2 );
	  //exec_function( 8, FunctionPin8, (FuncState & FN_BIT_08)>>3 );
	  break;
	  
  case FN_9_12:
	  //exec_function( 9, FunctionPin9,   (FuncState & FN_BIT_09));
      //exec_function( 10, FunctionPin10, (FuncState & FN_BIT_10)>>1 );
      //exec_function( 11, FunctionPin11, (FuncState & FN_BIT_11)>>2 );
  	  //exec_function( 12, FunctionPin12, (FuncState & FN_BIT_12)>>3 );
	  break;
  case FN_13_20:   //Function Group 2 FuncState == F20-F13 Function Control
      //exec_function( 13, FunctionPin13, (FuncState & FN_BIT_13);
	  //exec_function( 14, FunctionPin14, (FuncState & FN_BIT_14)>>1;
 	  //exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
 	  //exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
      break;
	
  case FN_21_28:
      break;	
  }
}  // end notifyDccFunc

void exec_function (int function, int pin, int FuncState)  {
#ifdef DEBUG
   Serial.print("function= ");
   Serial.println(function, DEC) ;
   Serial.print("FuncState= ");
   Serial.println(FuncState, DEC) ;
#endif
  switch ( Dcc.getCV( 30+function) )  {  // Config 0=On/Off,1=Blink
     case 0:    // Master Disable by Function 0
	    MasterDisable_value = byte(FuncState);
      break;
    case 1:    // Master Disable by Function 1
	    Disable_welder1 = byte(FuncState);
      break;
    case 2:    // Next Features									   
      break;  
    default:
      break;
  }
}  // end exec_function
