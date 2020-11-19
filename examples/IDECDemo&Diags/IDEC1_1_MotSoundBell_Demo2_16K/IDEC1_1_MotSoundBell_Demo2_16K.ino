// Interactive Decoder Motor, Pauses, Reversals w/Sound 4 LED IDEC1_1_MotSound4Led.ino
// Version 1.08 Geoff Bunza 2020
// Works with both short and long DCC Addesses
// This decoder uses switches/sensors to control 2 motors and Five LEDs with Sound
// F0=Master Function OFF = Function ON DISABLES the decoder
// Input Pin for Decoder Disable  Pin 16 Active LOW
//Motor speed via DCC speed for one motor, second motor on/off via function
//Speed Over-Ride =  CV = Non-Zero Value (1-127) over-rides the DCC speed commands Bit 8 is direction 1=Forward
//Input1 Pin for Throttle Down/Pause/Throttle Up  Pin 5 
//         CV for Throttle Down Time, CV for Throttle Up Time,, CV for Pause Time
//Input2 Pin for Throttle Down/Pause/Throttle Up  Pin 6
//          CV for Throttle Down Time, CV for Throttle Up Time,, CV for Pause Time
//Input Pin1 for Throttle Down/Reverse/Throttle Up Pin 7
//    CV for Throttle Down Time, CV for Throttle Up Time;,CV for Reverse Pause Time
//Input Pin2 for Throttle Down/Reverse/Throttle Up Pin 8
//    CV for Throttle Down Time, CV for Throttle Up Time;,CV for Reverse Pause Time
//Input Pin for immediate Stop  Pin 11
//Input Pin for Immediate Start Pin 12
//Functions for lights on/off:
// F1-F5  5 Functions LED ON/OFF by default PINS 13,14,17,18,19
/* Pro Mini  D15 A1 (TX) connected to  DFPlayer1 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
 *  Remember to connect +5V and GND to the DFPlayer too: DFPLAYER PINS 1 & 7 respectively
 *  This is a “mobile/function” decoder with audio play to dual motor control and 
 *  LED functions. Audio tracks or clips are stored on a micro SD card for playing, 
 *  in a folder labeled mp3, with tracks named 0001.mp3, 0002.mp3, etc. 
 * MAX 3 Configurations per pin function:
 *  Config 0=Decoder DISABLE On/Off, 1=LED; 2=Motor2 Control On/Off
  F0 == Master Decoder Disable == ON
  F1 == LED == D13
  F2 == LED == D14/A0
  F3 == LED == D17/A3
  F4 == LED == D18/A4
  F5 == LED == D19/A5
  F6 == Motor2 On/OFF speed & direction set by CV 80  Normally Base Station will Transmit F5 as initial OFF
           If no DCC present Decoder will power up with Motor2 ON at speed specified in CV 72
  Motor1 speed control is via throttle or overridden by non zero value in CV 50 
           High Bit=Direction, Lower 7 Bits=Speed == DSSSSSSS
 
PRO MINI PIN ASSIGNMENT:
2 - DCC Input
3 - m2h Motor Control
4 - m2l Motor Control
5 - Input1 Pin for Throttle Down/Pause/Throttle Up 
6 - Input2 Pin for Throttle Down/Pause/Throttle Up
7 - Input1 Pin for Throttle Down/Reverse/Throttle Up
8 - Input2 Pin for Throttle Down/Reverse/Throttle Up
9 - m0h Motor Control
10 - m0l Motor Control
11 - Input Pin for immediate Stop
12 - Input Pin for Immediate Start 
13 - LED F1
14 A0 - LED F2
15 A1 - (TX) connected to  DFPlayer1 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
16 A2 - Input Pin for MasterDecoderDisable Active LOW
17 A3 - LED F3
18 A4 - LED F4
19 A5 - LED F5
*/

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE PAUSE 1 SENSOR
//#define Pause1

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE PAUSE 2 SENSOR
//#define Pause2

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE REVERSE 1 SENSOR
#define Reverse1

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE REVERSE 2 SENSOR
//#define Reverse2

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE IMMEDIATE STOP SENSOR
//#define ImmediateStop

// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO INCLUDE THE IMMEDIATE START SENSOR
//#define ImmediateStart

#include <NmraDcc.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
SoftwareSerial DFSerial1(21,15); // PRO MINI RX, PRO MINI TX serial to DFPlayer
DFRobotDFPlayerMini Player1;

#define This_Decoder_Address 24
uint8_t CV_DECODER_MASTER_RESET = 252;

//Uncomment ONLY ONE of the following:
//#define  MasterTimeConstant  10L      // 10's of milliseconds Timing
#define  MasterTimeConstant  100L     // Tenths of a second Timing
//#define  MasterTimeConstant  1000L    // Seconds Timing
//#define  MasterTimeConstant  10000L   // 10's of Seconds Timing
//#define  MasterTimeConstant  60000L   // Minutes Timing
//#define  MasterTimeConstant  3600000L // Hours Timing

uint16_t ttemp, i;
#define First_Track 1   // Play Random Tracks First_Track#=Start_Track  >=1
#define Last_Track  2   // Play Random Tracks Last_Track= Last Playable Track in Range  <= Last Numbered Track
#define starting_volume  22   // If no volume is set use this at the start
const int audiocmddelay = 34;
 
boolean Use_DCC_speed = true; // Switch to disable DCC Speed updates
int Motor1Speed          = 0;   //  Variablw for Motor1 Speed
int Starting_Motor1Speed = 0;
int Motor1ForwardDir  = 1;      //  Variable for Motor1 Dir
int ForcedStopSpeedMotor1 = 0;  // Holding Variablw for Last Speed when Immediate Stop
int ForcedStopDirMotor1 = 1;    // Holding Variablw for Last Direction when Immediate Stop
int Motor2Speed       = 0;      //  Variable for Motor2 Speed
int Motor2ForwardDir  = 1;      //  Variable for Motor2 Dir
int Motor2ON = 0;
int cyclewidth = 16384;

const int m2h = 3;              //R H Bridge    Motor1
const int m2l = 4;              //B H Bridge    Motor1
const int ThrottlePause1Pin = 5;       // Throttle Speed Pause1 Input Pin
const int ThrottlePause2Pin = 6;       //  Throttle Speed Pause2 Input Pin
const int ThrottleInputReverse1Pin = 7; //  Throttle Speed Reverse Input Pin
const int ThrottleInputReverse2Pin = 8;     //  Throttle Immediate Speed Reverse Input Pin
const int m0h = 9;                     //R H Bridge Motor2
const int m0l = 10;                    //B H Bridge     //Motor2
const int ImmediateStopPin = 11;       //  Throttle Immediate Stop Input Pin
const int ImmediateStartPin = 12;      //  Throttle Immediate Start Input Pin
const int MasterDecoderDisablePin = 16; // D16/A0  Master Decoder Disable Input Pin Active LOW
//  arduino pin D 15;    // D15/A1 DFPlayer Receive  (RX)  Pin 2 via 470 Ohm Resistor

const int numfpins = 10;              // Number of Output pins to initialize
const int num_active_functions = 7;   // Number of Functions stating with F0
byte fpins [] = {13,13,14,17,18,19,3,4,9,10};  //These are all the Output Pins (first 13 is placeholder)
const int FunctionPin0 = 20;    // D14/A0 DFPlayer Transmit (TX)  Pin 3
const int FunctionPin1 = 13;    // A2 LED
const int FunctionPin2 = 14;    // A3 LED
const int FunctionPin3 = 17;    // A4 LED
const int FunctionPin4 = 18;    // A5 LED
const int FunctionPin5 = 19;    // A6 LED 

const int FunctionPin6  = 20;   // Turns on Motor2 DCC Function Control Only NO Local Input Pin
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
int Function0_value = 0;

NmraDcc  Dcc ;
DCC_MSG  Packet ;
int t;  // temp
struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};
QUEUE *ftn_queue = new QUEUE[17];

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
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

  {30, 0}, //F0 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  
  {31, 1}, //F1 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {32, 1}, //F2 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {33, 1}, //F3 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {34, 1}, //F4 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {35, 1}, //F5 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  
  {36, 2}, //F6 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  
  {37,4}, //F7 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {38,4}, //F8 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {39,4}, //F9 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {40,4}, //F10 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {41,4}, //F11 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {42,4}, //F12 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {43,4}, //F13 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {44,4}, //F14 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {45,4}, //F15 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {46,4}, //F16 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {47,4}, //F17 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {48,4}, //F18 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  {49,4}, //F19 Config 0=DISABLE On/Off,1=LED,2=Motor2 Control On/Off,3=NOT Implemented
  
  {50, 0},  // Speed Over-Ride =  CV = Non-Zero Value (1-127) over-rides the DCC speed commands
            // Bit 8 (128 or 0x80) ON=Forward Direction 0=Reverse Direction
      
  {51, 0}, // ThrottlePause1 Pause Time 0-255 (0.1 secs)
  {52, 0}, // ThrottlePause1 Throttle Ramp DOWN Delay 0-255 Larger Delay=Slower Ramp Down
  {53, 0}, // ThrottlePause1 Throttle Ramp UP Delay 0-255  Larger Delay=Slower Ramp Up
  {54, 11}, // ThrottlePause1 Pause Sound Clip 1-nn 0=No Sound
  {55, 55}, // ThrottlePause1 Pause Sound Clip Volume 0-30
  
  {56, 0}, // ThrottlePause2 Pause Time 0-255 (0.1 secs)
  {57, 0}, // ThrottlePause2 Throttle Ramp DOWN 0-255 Delay
  {58, 0}, // ThrottlePause2 Throttle Ramp UP Delay 0-255
  {59, 11}, // ThrottlePause2 Pause Sound Clip 1-nn 0=No Sound
  {60, 55}, // ThrottlePause2 Pause Sound Clip Volume 0-30
  
  {61, 0}, // ThrottleInputReverse1 Pause Time 0-255 (0.1 secs)
  {62, 0}, // ThrottleInputReverse1 Ramp DOWN Delay 0-255
  {63, 0}, // ThrottleInputReverse1 Ramp UP Delay 0-255 
  {64, 11}, // ThrottleInputReverse1 Sound Clip 1-nn 0=No Sound
  {65, 55}, // ThrottleInputReverse1 Sound Clip Volume 0-30
  
  {66, 0}, // ThrottleInputReverse2 Pause Time 0-255 (0.1 secs)
  {67, 0}, // ThrottleInputReverse2 Ramp DOWN Delay 0-255
  {68, 0}, // ThrottleInputReverse2 Ramp UP Delay 0-255 
  {69, 11}, // ThrottleInputReverse2 Sound Clip 1-nn 0=No Sound
  {70, 55}, // ThrottleInputReverse2 Sound Clip Volume 0-30
  
  {71, 0},  // ThrottleImmediateStop Sound Clip 1-nn 0=No Sound
  {72, 55}, // ThrottleImmediateStop Sound Clip Volume 0-30
  
  {73, 0},  // ThrottleImmediateStart Sound Clip 1-nn 0=No Sound
  {74, 55}, // ThrottleImmediateStart Sound Clip Volume 0-30

  {80, 0}, // Motor2 Speed 0-127 Bit 8 (128 or 0x80) ON=Forward Direction 0=Reverse Direction

  //252,252    CV_DECODER_MASTER_RESET
  
  {253, 0},  // Extra
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

void setup()   //******************************************************
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  DFSerial1.begin (9600);
  Player1.begin (DFSerial1);
  
  pinMode (ThrottlePause1Pin,INPUT_PULLUP);        // Throttle Speed Pause1 Input Pin Active LOW
  pinMode (ThrottlePause2Pin,INPUT_PULLUP);        // Throttle Speed Pause2 Input Pin Active LOW
  pinMode (ThrottleInputReverse1Pin,INPUT_PULLUP); // Throttle Speed Reverse Input Pin 1 Active LOW
  pinMode (ThrottleInputReverse2Pin,INPUT_PULLUP); // Throttle Speed Reverse Input Pin 2 Active LOW
  pinMode (ImmediateStopPin,INPUT_PULLUP);         // Throttle Immediate Stop Input Pin Active LOW
  pinMode (ImmediateStartPin,INPUT_PULLUP);        // Throttle Immediate Start Input Pin Active LOW
  pinMode (MasterDecoderDisablePin,INPUT_PULLUP);  //  Master Decoder Disable Input Pin Active LOW
  uint8_t cv_value;
  // initialize the digital pins as outputs
    for (int i=0; i < numfpins; i++) {
      pinMode(fpins[i], OUTPUT);
      digitalWrite(fpins[i], 0);    // All OUPUT pins initialized LOW
     } 
  // Setup which External Interrupt, the Pin it's associated with that we're using 
  Dcc.pin(0, 2, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 61, FLAGS_MY_ADDRESS_ONLY, 0 );
  delay(800);
#if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET ) 
#endif 
     {
       for (int j=0; j < FactoryDefaultCVIndex; j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
     }
  for ( i=0; i < num_active_functions; i++) {
    cv_value = Dcc.getCV(30+i) ;   
    switch ( cv_value ) {
      case 0:   // Master Decoder Disable
        MasterDecoderDisable = 0;
        if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
        break;
      case 1:   // LED On/Off
          ftn_queue[i].inuse = 0;
        break;       
       case 2:   // Motor2 Control
         if ( Dcc.getCV(72) != 0) {
           Motor2ON = 1;
           Motor2Speed = (Dcc.getCV(72))&0x7f ;
           Motor2ForwardDir = (byte)((Dcc.getCV(72))&0x80)>>7 ;
         } else  Motor2ON = 0;
       break;
     case 3:   // NEXT FEATURE for the Future
       break;  
     default:
       break;
    }
  }
  setVolumeOnChannel (starting_volume);
  Motor1ForwardDir = 1;     // Default start value for direction if throttle controlled
  if ( Dcc.getCV(50) != 0) {
    Motor1Speed = (Dcc.getCV(50))&0x7f ;
    Motor1ForwardDir  = (byte)((Dcc.getCV(50))&0x80 )>>7;
  }
#ifdef DEBUG
    Serial.println("CV Dump:");
    for (i=30; i<51; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
	
    Serial.println("Throttle Pause 1");
    for (i=51; i<56; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");

    Serial.println("Throttle Pause 2");
    for (i=56; i<61; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
	
	Serial.println("Throttle Reverse 1");
    for (i=61; i<66; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
	
	Serial.println("Throttle Reverse 2");
    for (i=66; i<71; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
	
	Serial.println("Immediate Stop");
    for (i=71; i<73; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
	
	Serial.println("Immediate Start");
    for (i=73; i<75; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    
	Serial.println("Motor2 Speed");
    Serial.print(Dcc.getCV(80),DEC); Serial.print("\t"); }
    Serial.println("");
#endif
}
void loop()   //**********************************************************************
{
  //MUST call the NmraDcc.process() method frequently 
  // from the Arduino loop() function for correct library operation
  //Dcc.process();
  run_at_speed();
  //delay(1);

//  INPUT OVER RIDES
   // Check Master Input Over ride
   MasterDecoderDisable = 0;
   if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
     else MasterDecoderDisable = Function0_value & 1;
   if (MasterDecoderDisable == 1) { Motor1Speed = 0; Motor2Speed = 0; }

#ifdef Pause1
// ========   Throttle Pause 1  ========================
   if (digitalRead(ThrottlePause1Pin) == LOW) {    // Throttle Speed Pause1 Input Pin
     Use_DCC_speed = false;              // Do not update speed via DCC
     Starting_Motor1Speed = Motor1Speed;
     while (Motor1Speed >0) {
       --Motor1Speed;
       run_at_speed();
       delay(Dcc.getCV(52));                     //Throttle Ramp DOWN Delay 0-255  
     }
     Motor1Speed = 0;
     ttemp=(Dcc.getCV(54));
     setVolumeOnChannel (Dcc.getCV(55));
     if (ttemp!=0) playTrackOnChannel(ttemp);      //  play clip 
     delay(int(Dcc.getCV(51)*MasterTimeConstant)); //Pause Time 0-255 (0.1 secs)
     while (Motor1Speed <= Starting_Motor1Speed) {
       ++Motor1Speed;
       run_at_speed();
       delay(Dcc.getCV(53));                     //Throttle Ramp UP Delay 0-255  
     }
     Motor1Speed = Starting_Motor1Speed;
     for (i=0; i<30; i++) run_at_speed();      // Move away from sensor
     while (digitalRead(ThrottlePause1Pin) == LOW) run_at_speed(); //Wait for Sensor
     Use_DCC_speed = true;              // Do not update speed via DCC
   }
#endif

#ifdef Pause2
// ========   Throttle Pause 2  ========================
   if (digitalRead(ThrottlePause2Pin) == LOW) {    //  Throttle Speed Pause2 Input Pin
     
     Use_DCC_speed = false;              // Do not update speed via DCC
     Starting_Motor1Speed = Motor1Speed;
     while (Motor1Speed >0) {
       --Motor1Speed;
         run_at_speed();
       delay(Dcc.getCV(57));                     //Throttle Ramp DOWN Delay 0-255  
     }
     Motor1Speed = 0;
     ttemp=(Dcc.getCV(59));
     setVolumeOnChannel (Dcc.getCV(60));
     if (ttemp!=0) playTrackOnChannel(ttemp);      //  play clip 
     delay(int(Dcc.getCV(56)*MasterTimeConstant));                //Pause Time 0-255 (0.1 secs)
     while (Motor1Speed <= Starting_Motor1Speed) {
       ++Motor1Speed;
       run_at_speed();
       delay(Dcc.getCV(58));                     //Throttle Ramp UP Delay 0-255  
     }
     Motor1Speed = Starting_Motor1Speed;
     for (i=0; i<30; i++) run_at_speed();      // Move away from sensor
     while (digitalRead(ThrottlePause2Pin) == LOW) run_at_speed(); //Wait for Sensor
     Use_DCC_speed = true;              // Do not update speed via DCC
   }
#endif

#ifdef Reverse1
// ========   Throttle Reverse 1  ========================
   if (digitalRead(ThrottleInputReverse1Pin)==LOW){ //  Throttle Speed Reverse1 Input Pin
     Use_DCC_speed = false;              // Do not update speed via DCC
     Starting_Motor1Speed = Motor1Speed;
     Motor1Speed--;
     while (Motor1Speed >1) {
       run_at_speed();
       --Motor1Speed;
       if (Dcc.getCV(62)!=0) delay(Dcc.getCV(62));  //Throttle Ramp DOWN Delay 0-255
       else  Motor1Speed=0;
     }
     //Motor1Speed = 0;
     ttemp=(Dcc.getCV(64));
     if (ttemp!=0) {setVolumeOnChannel (Dcc.getCV(65)); playTrackOnChannel(ttemp);}     //  play clip 
     Motor1ForwardDir = (Motor1ForwardDir^0x01) & 0x01;
     delay(Dcc.getCV(61)*MasterTimeConstant);        //Pause Time 0-255 (0.1 secs)
     while (Motor1Speed < Starting_Motor1Speed) {
       Motor1Speed++;;
       run_at_speed();
       if (Dcc.getCV(63)!=0) delay(Dcc.getCV(63));   //Throttle Ramp UP Delay 0-255
       else  Motor1Speed=Starting_Motor1Speed; 
     }
     //Motor1Speed = Starting_Motor1Speed;
     for (i=0; i<10; i++) run_at_speed();      // Move away from sensor
     while (digitalRead(ThrottleInputReverse1Pin) == LOW) run_at_speed(); //Wait for Sensor
     Use_DCC_speed = true;
   }
#endif

#ifdef Reverse2
// ========   Throttle Reverse 2  ========================
   if (digitalRead(ThrottleInputReverse2Pin)==LOW){ //  Throttle Speed Reverse Input Pin
     Use_DCC_speed = false;              // Do not update speed via DCC
     Starting_Motor1Speed = Motor1Speed;
     while (Motor1Speed >0) {
       --Motor1Speed;
       run_at_speed();
       delay(Dcc.getCV(67));                     //Throttle Ramp DOWN Delay 0-255  
     }
     Motor1Speed = 0;
     ttemp=(Dcc.getCV(69));
     setVolumeOnChannel (Dcc.getCV(70));
     if (ttemp!=0) playTrackOnChannel(ttemp);      //  play clip 
     Motor1ForwardDir = (Motor1ForwardDir^0x01) & 0x01;
     delay(int(Dcc.getCV(66)*MasterTimeConstant));                //Pause Time 0-255 (0.1 secs)
     while (Motor1Speed <= Starting_Motor1Speed) {
       ++Motor1Speed;
       run_at_speed();
       delay(Dcc.getCV(68));                     //Throttle Ramp UP Delay 0-255  
     }
     Motor1Speed = Starting_Motor1Speed;
     for (i=0; i<30; i++) run_at_speed();      // Move away from sensor
     while (digitalRead(ThrottleInputReverse2Pin) == LOW) run_at_speed(); //Wait for Sensor
     Use_DCC_speed = true;              // Do not update speed via DCC
   }
#endif

#ifdef ImmediateStop
// ========   Throttle Immediate Stop  ========================
   if (digitalRead(ImmediateStopPin) == LOW) {     //  Throttle Immediate Stop Input Pin
     ForcedStopSpeedMotor1 = Motor1Speed;
     ForcedStopDirMotor1 = Motor1ForwardDir;
     Motor1Speed = 0;
     ttemp=(Dcc.getCV(71));
     setVolumeOnChannel (Dcc.getCV(72));
     if (ttemp!=0) playTrackOnChannel(ttemp);      //  play clip 
   }
#endif

#ifdef ImmediateStart
// ========   Throttle Immediate Start  ========================
   if (digitalRead(ImmediateStartPin) == LOW) {    //  Throttle Immediate Start Input Pin
     ttemp=(Dcc.getCV(73));
     setVolumeOnChannel (Dcc.getCV(74));
     if (ttemp!=0) playTrackOnChannel(ttemp);      //  play clip 
     if (ForcedStopSpeedMotor1 != 0)  {
        Motor1Speed = ForcedStopSpeedMotor1 ;
        Motor1ForwardDir  = ForcedStopDirMotor1;
     } 
     else
         if ( Dcc.getCV(50) != 0) {
            Motor1Speed = (Dcc.getCV(50))&0x7f ;
            Motor1ForwardDir  = (byte)((Dcc.getCV(50))&0x80 )>>7;
         }
     ForcedStopSpeedMotor1 = 0;   // Take us out of forced stop mode
     for (i=0; i<30; i++) run_at_speed();      // Move away from sensor
     while (digitalRead(ImmediateStartPin) == LOW) run_at_speed(); //Wait for Sensor
   }
#endif
//  ********************************************************************************
  
  for (int i=1; i < num_active_functions; i++) {   
    switch (Dcc.getCV(30+i)) {
      case 0:   //   Master Decoder Disable Ops
        break;
      case 1:   //   LED On/Off
      if (MasterDecoderDisable == 1) digitalWrite(fpins[i], 0); //decoder disabled so LEDs off
        break;      
      case 2:   // Motor2 Control
        Motor2Speed = (Dcc.getCV(72))&0x7f ;                  // Re-read Motor2Speed if the CV was updated
        Motor2ForwardDir = (byte)((Dcc.getCV(72))&0x80)>>7 ;  // Re-read Motor2ForwardDir if the CV was updated

        if ((MasterDecoderDisable == 0)&&(Motor2ON == 1)) {
          if (Motor2ForwardDir == 0)  gofwd2 (Motor2Speed<<4);
          else  gobwd2 (Motor2Speed<<4);
        }
        if (MasterDecoderDisable == 1) {
        digitalWrite(m0h, LOW);     //Motor2OFF
        digitalWrite(m0l, LOW);     //Motor2 OFF
        }
        break;
       
      case 3:   // NEXT FEATURE for the Future
        break;  
      default:
        break;  
      }
      run_at_speed();
  }
}         //  end loop()

void run_at_speed()  {
  Dcc.process();
  if (MasterDecoderDisable == 0)  {
    if (Motor1Speed != 0) { 
      if (Motor1ForwardDir == 0)  gofwd1 (Motor1Speed<<7);
      else gobwd1 (Motor1Speed<<7);
    }
  }
  if (MasterDecoderDisable == 1)  {
    digitalWrite(m2h, LOW);     //Motor1 OFF
    digitalWrite(m2l, LOW);     //Motor1 OFF
    digitalWrite(m0h, LOW);     //Motor2 OFF
    digitalWrite(m0l, LOW);     //Motor2 OFF
  }
  if ((MasterDecoderDisable == 0)&&(Motor2ON == 1)) {
          if (Motor2ForwardDir == 0)  gofwd2 (Motor2Speed<<7);
          else  gobwd2 (Motor2Speed<<7);
        }
}     //    end  run_at_speed()

void gofwd1(int fcycle) {
    digitalWrite(m2h, HIGH);     //Motor1
    delayMicroseconds(fcycle);
    digitalWrite(m2h, LOW);      //Motor1
    delayMicroseconds(cyclewidth-fcycle);
}     //  end   gofwd1()

void gobwd1(int bcycle) {
    digitalWrite(m2l, HIGH);     //Motor1
    delayMicroseconds(bcycle); 
    digitalWrite(m2l, LOW);      //Motor1
    delayMicroseconds(cyclewidth-bcycle);
}      //  end   gobwd1()

void gofwd2(int fcycle) {
    digitalWrite(m0h, HIGH);     //Motor2
    delayMicroseconds(fcycle);
    digitalWrite(m0h, LOW);      //Motor2
    delayMicroseconds(cyclewidth-fcycle);
}      //    end   gofwd2()

void gobwd2(int bcycle) {
    digitalWrite(m0l, HIGH);     //Motor2
    delayMicroseconds(bcycle); 
    digitalWrite(m0l, LOW);      //Motor2
    delayMicroseconds(cyclewidth-bcycle);
}     //    end gobwd2()

void playTrackOnChannel ( byte dtrack)  {
      if (dtrack!=255) {Player1.play(dtrack); } //delay(audiocmddelay); }
	    else {Player1.play(random(First_Track,Last_Track+1));}  // delay(audiocmddelay);
}     //    end   playTrackOnChannel()

void setVolumeOnChannel ( byte dvolume)  {
      if(dvolume>30) return;    // Don't change the volume if out of range
      Player1.volume (dvolume);
      delay(audiocmddelay);
}     //    end   setVolumeOnChannel()

void    notifyCVChange( uint16_t CV, uint8_t Value)  {
  if ( CV== 50 )  {
    Motor1Speed = (Dcc.getCV(50))&0x7f ;
    Motor1ForwardDir  = (byte)((Dcc.getCV(50))&0x80 )>>7;
  }
}       //   end notifyCVChange()

void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION ForwardDir, DCC_SPEED_STEPS SpeedSteps )  {
     if ( !Use_DCC_speed )  return;
     if ( Dcc.getCV(50) == 0) {
       Motor1Speed = (Speed & 0x7f );
     }
     if (Motor1Speed == 1)  Motor1Speed = 0;
}        //   end   notifyDccSpeed()

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
    exec_function( 3, FunctionPin3, (FuncState & FN_BIT_03)>>2 );
    exec_function( 4, FunctionPin4, (FuncState & FN_BIT_04)>>3 );
      break;
  case FN_5_8:    //Function Group 1 S FFFF == 1 F8 F7 F6 F5  &  == 0  F12 F11 F10 F9 F8
    exec_function( 5, FunctionPin5, (FuncState & FN_BIT_05));
    exec_function( 6, FunctionPin6, (FuncState & FN_BIT_06)>>1 );
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
}       //   end notifyDccSpeed()

void exec_function (int function, int pin, int FuncState)  {
#ifdef DEBUG
   Serial.print("ex function= ");
   Serial.println(function, DEC) ;
   Serial.print("FuncState= ");
   Serial.println(FuncState, DEC) ;
#endif
  switch ( Dcc.getCV( 30+function) )  {  // Config 0=On/Off,1=Blink
    case 0:    // Master Disable Function0 Value will transfer to MasterDecoderDisable in loop()
      Function0_value = byte(FuncState);
      break;
    case 1:    // On - Off LED
    if (MasterDecoderDisable == 0)  {
        digitalWrite (pin, FuncState);
    }
      break;
    case 2:    // Motor2 Control
    if (MasterDecoderDisable == 0) Motor2ON= FuncState;
      break;
    case 3:    // NEXT FEATURE for the Future
      break;  
    default:
      ftn_queue[function].inuse = 0;
      break;
  }
}    //   end  exec_function()

/*  DFPlayer Commands
//----Set volume----
  myDFPlayer.volume(10);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
//  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);
  //----Mp3 control----
//  myDFPlayer.sleep();     //sleep
//  myDFPlayer.reset();     //Reset the module
//  myDFPlayer.enableDAC();  //Enable On-chip DAC
//  myDFPlayer.disableDAC();  //Disable On-chip DAC
//  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
  //----Mp3 play----
  myDFPlayer.next();  //Play next mp3
  myDFPlayer.previous();  //Play previous mp3
  myDFPlayer.play(1);  //Play the first mp3
  myDFPlayer.loop(1);  //Loop the first mp3
  myDFPlayer.pause();  //pause the mp3
  myDFPlayer.start();  //start the mp3 from the pause
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  myDFPlayer.stopAdvertise(); //stop advertise
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  myDFPlayer.randomAll(); //Random play all the mp3.
  myDFPlayer.enableLoop(); //enable loop.
  myDFPlayer.disableLoop(); //disable loop.
*/
