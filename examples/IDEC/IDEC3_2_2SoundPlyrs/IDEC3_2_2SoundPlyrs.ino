// Interactive Decoder Sounds and Lights Two Sound Players  IDEC3_2_2SoundPlyrs.ino
// Version 1.08 Geoff Bunza 2020
// Works with both short and long DCC Addesses
// This decoder will control Sound Sequences and LEDs
// F0=Master Function OFF = Function ON DISABLES the decoder
// Input Pin for Decoder Disable  Pin 16/A2 Active LOW
//Functions for lights on/off:
// F11-F14 Four Functions LED ON/OFF by default PINS 13,16,17,18,19
/*  
 *  Pro Mini  D14 A0 (TX) connected to  DFPlayer2 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
 *  Pro Mini  D15 A1 (TX) connected to  DFPlayer1 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
 *  Remember to connect +5V and GND to the DFPlayer too: DFPLAYER PINS 1 & 7 respectively
 *  This is a “mobile/function” decoder with audio play to dual motor control and 
 *  LED functions. Audio tracks or clips are stored on a micro SD card for playing, 
    in a folder labeled mp3, with tracks named 0001.mp3, 0002.mp3, etc. 
 * Up to 10 Sound Sets can be defined (and programmed via CVs) each sound set consists of:
    delay1,volume1,sound clip1,delay2,volume2,sound clip2,delay3,volume3,sound clip3,delay4,volume4,sound clip4,delay5,volume5,sound clip5,PlayerCnannel (0-1) -- Timing of playing the sound set is completely independent of other sound sets and can occur simultaneously
    If a sound clip/track is started while another clp/track is playing on the same player channel, the newly starting clip/track
 * Delay set to zero will offer NO delay time
 * Volume set to >30 will not change current volume setting
 * Track set to 0 will skip playing a track -- delay and volume set can be set to 0 and >30 to ignore operation too, 
     otherwise delay and volume are executed normally
 * Track set to 255 will select a random track to play from number First_Track to Last_Track INCLUSIVE
 *  F0 is configured as Master Decoder Disable Override  ON==Disable Decoder
 *  F1-F10  plays an audio set (1-10) defined by CVs
 * MAX one of 12 Configurations per pin function:
 *  Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
 * F0 Master Decoder Disable
 * F1-F10  Play Sound Set 1-10
 * F11-F14  Functions LED ON/OFF
 * Both F1-F10 set ON or the appropriate Input Pin 3-12 held LOW will repeat play the associated sound set
   until BOTH the respective DCC function goes OFF  AND the respective Input Pin goes HIGH
 
PRO MINI PIN ASSIGNMENT:
2 - DCC Input
3 - Input Pin Play Sound Set1
4 - Input Pin Play Sound Set2
5 - Input Pin Play Sound Set3
6 - Input Pin Play Sound Set4
7 - Input Pin Play Sound Set5
8 - Input Pin Play Sound Set6
9 - Input Pin Play Sound Set7
10 - Input Pin Play Sound Set8
11 - Input Pin Play Sound Set9
12 - Input Pin Play Sound Set10
13 - LED F1
14 A0 - DFPlayer2 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
15 A1 - DFPlayer1 Receive  (RX) Pin 2 via 1K Ohm 1/4W Resistor
16 A2 - Input Pin for MasterDecoderDisable Active LOW
17 A3 - LED F2
18 A4 - LED F3
19 A5 - LED F4
*/
// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
SoftwareSerial DFSerial2(21,14); // PRO MINI RX, PRO MINI TX serial to DFPlayer
SoftwareSerial DFSerial1(22,15); // PRO MINI RX, PRO MINI TX serial to DFPlayer
DFRobotDFPlayerMini Player1;
DFRobotDFPlayerMini Player2;

#define This_Decoder_Address 24
uint8_t CV_DECODER_MASTER_RESET = 252;

//Uncomment ONLY ONE of the following:
//#define  MasterTimeConstant  10L      // 10's of milliseconds Timing
#define  MasterTimeConstant  100L     // Tenths of a second Timing
//#define  MasterTimeConstant  1000L    // Seconds Timing
//#define  MasterTimeConstant  10000L   // 10's of Seconds Timing
//#define  MasterTimeConstant  60000L   // Minutes Timing
//#define  MasterTimeConstant  3600000L // Hours Timing

unsigned long estop = 1;
int del_tim = 4000;
uint16_t ttemp, i;
#define First_Track 1   // Play Random Tracks First_Track#=Start_Track  >=1
#define Last_Track 12   // Play Random Tracks Last_Track= Last Playable Track in Range  <= Last Numbered Track
const int audiocmddelay = 34;
byte ss1[] = {0,0,0,0,0,0};  unsigned long ss1delay=0;
byte ss2[] = {0,0,0,0,0,0};  unsigned long ss2delay=0;
byte ss3[] = {0,0,0,0,0,0};  unsigned long ss3delay=0;
byte ss4[] = {0,0,0,0,0,0};  unsigned long ss4delay=0;
byte ss5[] = {0,0,0,0,0,0};  unsigned long ss5delay=0;
byte ss6[] = {0,0,0,0,0,0};  unsigned long ss6delay=0;
byte ss7[] = {0,0,0,0,0,0};  unsigned long ss7delay=0;
byte ss8[] = {0,0,0,0,0,0};  unsigned long ss8delay=0;
byte ss9[] = {0,0,0,0,0,0};  unsigned long ss9delay=0;
byte ss10[] = {0,0,0,0,0,0};  unsigned long ss10delay=0;
bool playing_sound_set [ ] = {false,false,false,false,false,false,false,false,false,false,false};
byte soundset_channel[ ]={0,0,0,0,0,0,0,0,0,0,0};
const int MasterDecoderDisablePin = 16; // D16/A0  Master Decoder Disable Input Pin Active LOW

const int numINpins = 11;              // Number of INput pins to initialize
byte inputpins [] = {3,4,5,6,7,8,9,10,11,12,16};  //These are all the Input Pins
const int numfpins = 4;              // Number of Output pins to initialize
const int num_active_functions = 12;   // Number of Functions stating with F0
byte fpins [] = {13,17,18,19};  //These are all the Output Pins (first 15 is placeholder)
const int FunctionPin0 = 20;    // D14/A0 DFPlayer Transmit (TX)  Pin 3
const int FunctionPin1 = 20;    // A2 LED
const int FunctionPin2 = 20;    // A3 LED
const int FunctionPin3 = 20;    // A4 LED
const int FunctionPin4 = 20;    // A5 LED
const int FunctionPin5  = 20;   // Turns on Motor2 DCC Function Control Only NO Local Input Pin
const int FunctionPin6  = 20;   // Place holders ONLY
const int FunctionPin7  = 20;   // Place holders ONLY
const int FunctionPin8  = 20;   // Place holders ONLY
const int FunctionPin9  = 20;   // Place holders ONLY
const int FunctionPin10 = 20;   // Place holders ONLY
const int FunctionPin11 = 13;   // Place holders ONLY
const int FunctionPin12 = 17;   // Place holders ONLY
const int FunctionPin13 = 18;   // Place holders ONLY
const int FunctionPin14 = 19;   // Place holders ONLY
const int FunctionPin15 = 20;   // Place holders ONLY
const int FunctionPin16 = 20;   // Place holders ONLY
int MasterDecoderDisable = 0;
int Function0_value = 0;
byte function_value [ ] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t cv_value;

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

  {30, 0}, //F0 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  
  {31, 1}, //F1 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {32, 2}, //F2 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {33, 3}, //F3 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {34, 4}, //F4 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {35, 5}, //F5 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {36, 6}, //F6 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {37, 7}, //F7 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {38, 8}, //F8 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {39, 9}, //F9 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {40, 10}, //F10 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  
  {41, 11}, //F11 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {42, 11}, //F12 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {43, 11}, //F13 Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {44, 11}, //F14Config 0=DISABLE On/Off,1-10=Sound Set Control 1-10,11=LED On/Off
  {45, 22}, //F15 not used
 
  {50, 3}, // Wait1 0-254 0.1 Seconds      // SOUND SET 1
  {51, 22}, // Volume1 0-30  >30 == no volume change
  {52,  1}, // Sound Clip1
  {53,  6}, // Wait2 0-254 0.1 Seconds
  {54, 22},  // Volume2 0-30  >30 == no volume change
  {55,  2}, // Sound Clip2
  {56,  6}, // Wait3 0-254 0.1 Seconds
  {57, 20}, // Volume3 0-30  >30 == no volume change
  {58,  3}, // Sound Clip3
  {59,  6},  // Wait4 0-254 0.1 Seconds
  {60, 15}, // Volume4 0-30  >30 == no volume change
  {61, 11}, // Sound Clip4
  {62, 11}, // Wait5 0-254 0.1 Seconds
  {63, 11}, // Volume5 0-30  >30 == no volume change
  {64, 12},  // Sound Clip5
  {65, 0}, // Sound Set Channel == LSB 0/1
  
  {66, 2}, // Wait1 0-254 0.1 Seconds      // SOUND SET 2
  {67, 20}, // Volume1 0-30  >30 == no volume change
  {68, 4}, // Sound Clip1
  {69, 6}, // Wait2 0-254 0.1 Seconds
  {70, 20},  // Volume2 0-30  >30 == no volume change
  {71, 5}, // Sound Clip2
  {72, 6}, // Wait3 0-254 0.1 Seconds
  {73, 20}, // Volume3 0-30  >30 == no volume change
  {74, 6}, // Sound Clip3
  {75, 6},  // Wait4 0-254 0.1 Seconds
  {76, 20}, // Volume4 0-30  >30 == no volume change
  {77, 7}, // Sound Clip4
  {78, 6}, // Wait5 0-254 0.1 Seconds
  {79, 20}, // Volume5 0-30  >30 == no volume change
  {80, 8},  // Sound Clip5
  {81, 1}, // Sound Set Channel == LSB 0/1
  
  {82, 1}, // Wait1 0-254 0.1 Seconds      // SOUND SET 3
  {83, 20}, // Volume1 0-30  >30 == no volume change
  {84, 5}, // Sound Clip1
  {85, 6}, // Wait2 0-254 0.1 Seconds
  {86, 20},  // Volume2 0-30  >30 == no volume change
  {87, 6}, // Sound Clip2
  {88, 6}, // Wait3 0-254 0.1 Seconds
  {89, 20}, // Volume3 0-30  >30 == no volume change
  {90, 7}, // Sound Clip3
  {91, 6},  // Wait4 0-254 0.1 Seconds
  {92, 20}, // Volume4 0-30  >30 == no volume change
  {93, 8}, // Sound Clip4
  {94, 6}, // Wait5 0-254 0.1 Seconds
  {95, 20}, // Volume5 0-30  >30 == no volume change
  {96, 8},  // Sound Clip5
  {97, 0}, // Sound Set Channel == LSB 0/1

  {98, 0}, // Wait1 0-254 0.1 Seconds      // SOUND SET 4
  {99, 23}, // Volume1 0-30  >30 == no volume change
  {100, 9}, // Sound Clip1
  {101,110}, // Wait2 0-254 0.1 Seconds
  {102, 99},  // Volume2 0-30  >30 == no volume change
  {103, 0}, // Sound Clip2
  {104, 0}, // Wait3 0-254 0.1 Seconds
  {105, 20}, // Volume3 0-30  >30 == no volume change
  {106, 0}, // Sound Clip3
  {107, 0},  // Wait4 0-254 0.1 Seconds
  {108, 99}, // Volume4 0-30  >30 == no volume change
  {109, 0}, // Sound Clip4
  {110, 0}, // Wait5 0-254 0.1 Seconds
  {111, 99}, // Volume5 0-30  >30 == no volume change
  {112, 0},  // Sound Clip5
  {113, 0}, // Sound Set Channel == LSB 0/1
  
  {114, 0}, // Wait1 0-254 0.1 Seconds      // SOUND SET 5
  {115, 20}, // Volume1 0-30  >30 == no volume change
  {116, 1}, // Sound Clip1
  {117, 20}, // Wait2 0-254 0.1 Seconds
  {118, 20},  // Volume2 0-30  >30 == no volume change
  {119, 2}, // Sound Clip2
  {120, 30}, // Wait3 0-254 0.1 Seconds
  {121, 20}, // Volume3 0-30  >30 == no volume change
  {122, 3}, // Sound Clip3
  {123, 50},  // Wait4 0-254 0.1 Seconds
  {124, 20}, // Volume4 0-30  >30 == no volume change
  {125, 4}, // Sound Clip4
  {126, 100}, // Wait5 0-254 0.1 Seconds
  {127, 20}, // Volume5 0-30  >30 == no volume change
  {128, 5},  // Sound Clip5
  {129, 0}, // Sound Set Channel == LSB 0/1
  
  {130, 0}, // Wait1 0-254 0.1 Seconds      // SOUND SET 6
  {131, 20}, // Volume1 0-30  >30 == no volume change
  {132, 1}, // Sound Clip1
  {133, 20}, // Wait2 0-254 0.1 Seconds
  {134, 20},  // Volume2 0-30  >30 == no volume change
  {135, 2}, // Sound Clip2
  {136, 30}, // Wait3 0-254 0.1 Seconds
  {137, 20}, // Volume3 0-30  >30 == no volume change
  {138, 3}, // Sound Clip3
  {139, 50},  // Wait4 0-254 0.1 Seconds
  {140, 20}, // Volume4 0-30  >30 == no volume change
  {141, 4}, // Sound Clip4
  {142, 100}, // Wait5 0-254 0.1 Seconds
  {143, 20}, // Volume5 0-30  >30 == no volume change
  {144, 5},  // Sound Clip5
  {145, 1}, // Sound Set Channel == LSB 0/1
  
  {146, 10}, // Wait1 0-254 0.1 Seconds      // SOUND SET 7
  {147, 29}, // Volume1 0-30  >30 == no volume change
  {148, 1}, // Sound Clip1
  {149, 10}, // Wait2 0-254 0.1 Seconds
  {150, 24},  // Volume2 0-30  >30 == no volume change
  {151, 2}, // Sound Clip2
  {152, 10}, // Wait3 0-254 0.1 Seconds
  {153, 18}, // Volume3 0-30  >30 == no volume change
  {154, 3}, // Sound Clip3
  {155, 10},  // Wait4 0-254 0.1 Seconds
  {156, 12}, // Volume4 0-30  >30 == no volume change
  {157, 4}, // Sound Clip4
  {158, 10}, // Wait5 0-254 0.1 Seconds
  {159, 6}, // Volume5 0-30  >30 == no volume change
  {160, 5},  // Sound Clip5
  {161, 0}, // Sound Set Channel == LSB 0/1
  
  {162, 0}, // Wait1 0-254 0.1 Seconds      // SOUND SET 8
  {163, 20}, // Volume1 0-30  >30 == no volume change
  {164, 1}, // Sound Clip1
  {165, 20}, // Wait2 0-254 0.1 Seconds
  {166, 20},  // Volume2 0-30  >30 == no volume change
  {167, 2}, // Sound Clip2
  {168, 30}, // Wait3 0-254 0.1 Seconds
  {169, 20}, // Volume3 0-30  >30 == no volume change
  {170, 3}, // Sound Clip3
  {171, 50},  // Wait4 0-254 0.1 Seconds
  {172, 20}, // Volume4 0-30  >30 == no volume change
  {173, 4}, // Sound Clip4
  {174, 100}, // Wait5 0-254 0.1 Seconds
  {175, 20}, // Volume5 0-30  >30 == no volume change
  {176, 5},  // Sound Clip5
  {177, 1}, // Sound Set Channel == LSB 0/1
  
  {178, 0}, // Wait1 0-254 0.1 Seconds      // SOUND SET 9
  {179, 20}, // Volume1 0-30  >30 == no volume change
  {180, 255}, // Sound Clip1
  {181, 11}, // Wait2 0-254 0.1 Seconds
  {182, 20},  // Volume2 0-30  >30 == no volume change
  {183, 255}, // Sound Clip2
  {184, 11}, // Wait3 0-254 0.1 Seconds
  {185, 20}, // Volume3 0-30  >30 == no volume change
  {186, 255}, // Sound Clip3
  {187, 14},  // Wait4 0-254 0.1 Seconds
  {188, 20}, // Volume4 0-30  >30 == no volume change
  {189, 255}, // Sound Clip4
  {190, 14}, // Wait5 0-254 0.1 Seconds
  {191, 20}, // Volume5 0-30  >30 == no volume change
  {192, 255},  // Sound Clip5
  {193, 0}, // Sound Set Channel == LSB 0/1
  
  {194, 4}, // Wait1 0-254 0.1 Seconds      // SOUND SET 10
  {195, 20}, // Volume1 0-30  >30 == no volume change
  {196, 10}, // Sound Clip1
  {197, 110}, // Wait2 0-254 0.1 Seconds
  {198, 99},  // Volume2 0-30  >30 == no volume change
  {199, 0}, // Sound Clip2
  {200, 4}, // Wait3 0-254 0.1 Seconds
  {201, 99}, // Volume3 0-30  >30 == no volume change
  {202, 0}, // Sound Clip3
  {203, 4},  // Wait4 0-254 0.1 Seconds
  {204, 12}, // Volume4 0-30  >30 == no volume change
  {205, 9}, // Sound Clip4
  {206, 100}, // Wait5 0-254 0.1 Seconds
  {207, 99}, // Volume5 0-30  >30 == no volume change
  {208, 0},  // Sound Clip5
  {209, 1}, // Sound Set Channel == LSB 0/1

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
  DFSerial2.begin (9600);
  Player1.begin (DFSerial1);
  Player2.begin (DFSerial2);
  
  pinMode (MasterDecoderDisablePin,INPUT_PULLUP); //  Master Decoder Disable Input Pin Active LOW
  
  // initialize the digital pins as outputs
  for (int i=0; i < numfpins; i++) {
    pinMode(fpins[i], OUTPUT);
    digitalWrite(fpins[i], 0);    // All OUPUT pins initialized LOW
   }
  // initialize the digital pins as inputs
  for (int i=0; i < numINpins; i++) {
    pinMode(inputpins[i], INPUT_PULLUP);
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
   // Master Decoder Disable
   MasterDecoderDisable = 0;
   if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
#ifdef DEBUG
    Serial.println("CV Dump:");
    for (i=30; i<45; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=30; i<45; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 1");
    for (i=50; i<66; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=50; i<66; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 2");
    for (i=66; i<82; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=66; i<82; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 3");
    for (i=82; i<98; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=82; i<98; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 4");
    for (i=98; i<114; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=98; i<114; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 5");
    for (i=114; i<130; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=114; i<130; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 6");
    for (i=130; i<146; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=130; i<146; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 7");
    for (i=146; i<162; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=146; i<162; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 8");
    for (i=162; i<178; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=162; i<178; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 9");
    for (i=178; i<194; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=178; i<194; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Sound Set 10");
    for (i=194; i<210; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=194; i<210; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
#endif
}  //   End setup
void loop()   //***********************************************************************************
{
  //MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  delay(1);
//  INPUT OVER RIDES
   // Check Master Input Over ride
   MasterDecoderDisable = 0;
   if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
     else MasterDecoderDisable = Function0_value & 1; 
   if (MasterDecoderDisable == 1) {
     for (i=0; i < numfpins; i++) digitalWrite(fpins[i], 0); // All LEDs  set LOW
   }
//  ********************************************************************************
    if (MasterDecoderDisable == 0) {
      for (i=0; i < num_active_functions; i++) {
      cv_value = Dcc.getCV(30+i) ;   
#ifdef DEBUG
    //Serial.print(" cv_value: ");
    //Serial.println(cv_value, DEC) ;
#endif
      switch ( cv_value ) {
      case 0:   // Master Decoder Disable
      MasterDecoderDisable = 0;
        if (digitalRead(MasterDecoderDisablePin)==LOW) MasterDecoderDisable = 1;
      break;
      case 1:   // 
        if (((digitalRead(3)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
          ss1[0]=1; playing_sound_set[cv_value]=true; 
#ifdef DEBUG
    Serial.print(" cv_value: ");
    Serial.println(cv_value, DEC) ;
    Serial.print(" function_value[cv_value]: ");
    Serial.println(function_value[cv_value], DEC) ;
    Serial.print(" ss1[0]: ");
    Serial.println(ss1[0], DEC) ;
    Serial.print(" playing_sound_set[cv_value]: ");
    Serial.println(playing_sound_set[cv_value], DEC) ;
#endif
          }
      break;
      case 2:   //   
         if (((digitalRead(4)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss2[0]=1; playing_sound_set[cv_value]=true; 
#ifdef DEBUG
    Serial.print(" cv_value: ");
    Serial.println(cv_value, DEC) ;
    Serial.print(" function_value[cv_value]: ");
    Serial.println(function_value[cv_value], DEC) ;
    Serial.print(" ss2[0]: ");
    Serial.println(ss2[0], DEC) ;
    Serial.print(" playing_sound_set[cv_value]: ");
    Serial.println(playing_sound_set[cv_value], DEC) ;
#endif
           }

      break;
      case 3:   // 
          if (((digitalRead(5)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss3[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 4:   // 
          if (((digitalRead(6)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss4[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 5:   // 
          if (((digitalRead(7)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss5[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 6:   // 
          if (((digitalRead(8)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
          ss6[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 7:   // 
          if (((digitalRead(9)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss7[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 8:   // 
          if (((digitalRead(10)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss8[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 9:   // 
          if (((digitalRead(11)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
           ss9[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 10:   // 
          if (((digitalRead(12)==LOW)||(function_value[cv_value]==1)) && !playing_sound_set[cv_value]) {
          ss10[0]=1; playing_sound_set[cv_value]=true; }
      break;
      case 11:  //   LEDs
      break;
      default:
      break;
      }
    }
  }
  
   //    ==========================   Sound Set 1 Begin Play
  if (ss1[0]==1) {
     ss1delay=millis()+(long(Dcc.getCV(50)*MasterTimeConstant)); // Wait1
     ss1[0]=0;  ss1[1]=1;  
   soundset_channel[1]=Dcc.getCV(65); //load the channel in case it updated
    }
  if ((ss1[1]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(52));
#ifdef DEBUG
    Serial.print(" Here 1: ");
    Serial.println(ttemp, DEC) ;
#endif
    setVolumeOnChannel (Dcc.getCV(51),soundset_channel[1]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[1]);  //  play clip 1
    ss1delay=millis()+(long(Dcc.getCV(53)*MasterTimeConstant)); // Wait2
    ss1[1]=0;  ss1[2]=1;
    }
  if ((ss1[2]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(55));
#ifdef DEBUG
    Serial.print(" Here 2: ");
    Serial.println(ttemp, DEC) ;
#endif
      setVolumeOnChannel (Dcc.getCV(54),soundset_channel[1]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[1]);  //  play clip 2
      ss1delay=millis()+(long(Dcc.getCV(56)*MasterTimeConstant)); // Wait3
      ss1[2]=0;  ss1[3]=1;
    }
  if ((ss1[3]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(58));
#ifdef DEBUG
    Serial.print(" Here 3: ");
    Serial.println(ttemp, DEC) ;
#endif
      setVolumeOnChannel (Dcc.getCV(57),soundset_channel[1]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[1]);  //  play clip 3
      ss1delay=millis()+(long(Dcc.getCV(59)*MasterTimeConstant)); // Wait4
      ss1[3]=0;  ss1[4]=1;
    }
  if ((ss1[4]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(61));
      setVolumeOnChannel (Dcc.getCV(60),soundset_channel[1]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[1]);  //  play clip  4
      ss1delay=millis()+(long(Dcc.getCV(62)*MasterTimeConstant)); // Wait5
      ss1[4]=0;  ss1[5]=1;
    }
  if ((ss1[5]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(64));
      setVolumeOnChannel (Dcc.getCV(63),soundset_channel[1]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[1]);  //  play clip  5
      ss1[5]=0;    playing_sound_set[1]=false;
    }
//    ==========================   Sound Set 2 Begin Play
  if (ss2[0]==1) {
     ss2delay=millis()+(long(Dcc.getCV(66)*MasterTimeConstant)); // Wait1
     ss2[0]=0;  ss2[1]=1;
   soundset_channel[2]=Dcc.getCV(81); //load the channel in case it updated
    }
  if ((ss2[1]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(68));
    setVolumeOnChannel (Dcc.getCV(67),soundset_channel[2]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[2]);  //  play clip 1
    ss2delay=millis()+(long(Dcc.getCV(69)*MasterTimeConstant)); // Wait2
    ss2[1]=0;  ss2[2]=1;
    }
  if ((ss2[2]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(71));
      setVolumeOnChannel (Dcc.getCV(70),soundset_channel[2]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[2]);  //  play clip 2
      ss2delay=millis()+(long(Dcc.getCV(72)*MasterTimeConstant)); // Wait3
      ss2[2]=0;  ss2[3]=1;
    }
  if ((ss2[3]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(74));
      setVolumeOnChannel (Dcc.getCV(73),soundset_channel[2]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[2]);  //  play clip 3
      ss2delay=millis()+(long(Dcc.getCV(75)*MasterTimeConstant)); // Wait4
      ss2[3]=0;  ss2[4]=1;
    }
  if ((ss2[4]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(77));
      setVolumeOnChannel (Dcc.getCV(76),soundset_channel[2]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[2]);  //  play clip  4
      ss2delay=millis()+(long(Dcc.getCV(78)*MasterTimeConstant)); // Wait5
      ss2[4]=0;  ss2[5]=1;
    }
  if ((ss2[5]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(80));
      setVolumeOnChannel (Dcc.getCV(79),soundset_channel[2]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[2]);  //  play clip  5
      ss2[5]=0;    playing_sound_set[2]=false;
    }
//    ==========================   Sound Set 3 Begin Play
  if (ss3[0]==1) {
     ss3delay=millis()+(long(Dcc.getCV(82)*MasterTimeConstant)); // Wait1
     ss3[0]=0;  ss3[1]=1;
   soundset_channel[3]=Dcc.getCV(97); //load the channel in case it updated
    }
  if ((ss3[1]==1)&&(ss3delay<=millis())) {
    ttemp=(Dcc.getCV(84));
    setVolumeOnChannel (Dcc.getCV(83),soundset_channel[3]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[3]);  //  play clip 1
    ss3delay=millis()+(long(Dcc.getCV(85)*MasterTimeConstant)); // Wait2
    ss3[1]=0;  ss3[2]=1;
    }
  if ((ss3[2]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(87));
      setVolumeOnChannel (Dcc.getCV(86),soundset_channel[3]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[3]);  //  play clip 2
      ss3delay=millis()+(long(Dcc.getCV(88)*MasterTimeConstant)); // Wait3
      ss3[2]=0;  ss3[3]=1;
    }
  if ((ss3[3]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(90));
      setVolumeOnChannel (Dcc.getCV(89),soundset_channel[3]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[3]);  //  play clip 3
      ss3delay=millis()+(long(Dcc.getCV(91)*MasterTimeConstant)); // Wait4
      ss3[3]=0;  ss3[4]=1;
    }
  if ((ss3[4]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(93));
      setVolumeOnChannel (Dcc.getCV(92),soundset_channel[3]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[3]);  //  play clip  4
      ss3delay=millis()+(long(Dcc.getCV(94)*MasterTimeConstant)); // Wait5
      ss3[4]=0;  ss3[5]=1;
    }
  if ((ss3[5]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(96));
      setVolumeOnChannel (Dcc.getCV(95),soundset_channel[3]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[3]);  //  play clip  5
      ss3[5]=0;    playing_sound_set[3]=false;
    }
//    ==========================   Sound Set 4 Begin Play
  if (ss4[0]==1) {
     ss4delay=millis()+(long(Dcc.getCV(98)*MasterTimeConstant)); // Wait1
     ss4[0]=0;  ss4[1]=1;
   soundset_channel[4]=Dcc.getCV(113); //load the channel in case it updated
    }
  if ((ss4[1]==1)&&(ss4delay<=millis())) {
    ttemp=(Dcc.getCV(100));
    setVolumeOnChannel (Dcc.getCV(99),soundset_channel[4]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[4]);  //  play clip 1
    ss4delay=millis()+(long(Dcc.getCV(101)*MasterTimeConstant)); // Wait2
    ss4[1]=0;  ss4[2]=1;
    }
  if ((ss4[2]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(103));
      setVolumeOnChannel (Dcc.getCV(102),soundset_channel[4]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[4]);  //  play clip 2
      ss4delay=millis()+(long(Dcc.getCV(104)*MasterTimeConstant)); // Wait3
      ss4[2]=0;  ss4[3]=1;
    }
  if ((ss4[3]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(106));
      setVolumeOnChannel (Dcc.getCV(105),soundset_channel[4]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[4]);  //  play clip 3
      ss4delay=millis()+(long(Dcc.getCV(107)*MasterTimeConstant)); // Wait4
      ss4[3]=0;  ss4[4]=1;
    }
  if ((ss4[4]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(109));
      setVolumeOnChannel (Dcc.getCV(108),soundset_channel[4]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[4]);  //  play clip  4
      ss4delay=millis()+(long(Dcc.getCV(110)*MasterTimeConstant)); // Wait5
      ss4[4]=0;  ss4[5]=1;
    }
  if ((ss4[5]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(112));
      setVolumeOnChannel (Dcc.getCV(111),soundset_channel[4]); 
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[4]);  //  play clip  5
      ss4[5]=0;    playing_sound_set[4]=false;
    }
//    ==========================   Sound Set 5 Begin Play
  if (ss5[0]==1) {
     ss5delay=millis()+(long(Dcc.getCV(114)*MasterTimeConstant)); // Wait1
     ss5[0]=0;  ss5[1]=1;
   soundset_channel[5]=Dcc.getCV(129); //load the channel in case it updated
    }
  if ((ss5[1]==1)&&(ss5delay<=millis())) {
    ttemp=(Dcc.getCV(116));
    setVolumeOnChannel (Dcc.getCV(115),soundset_channel[5]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[5]);  //  play clip 1
    ss5delay=millis()+(long(Dcc.getCV(117)*MasterTimeConstant)); // Wait2
    ss5[1]=0;  ss5[2]=1;
    }
  if ((ss5[2]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(119));
      setVolumeOnChannel (Dcc.getCV(118),soundset_channel[5]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[5]);  //  play clip 2
      ss5delay=millis()+(long(Dcc.getCV(120)*MasterTimeConstant)); // Wait3
      ss5[2]=0;  ss5[3]=1;
    }
  if ((ss5[3]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(122));
      setVolumeOnChannel (Dcc.getCV(121),soundset_channel[5]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[5]);  //  play clip 3
      ss5delay=millis()+(long(Dcc.getCV(123)*MasterTimeConstant)); // Wait4
      ss5[3]=0;  ss5[4]=1;
    }
  if ((ss5[4]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(125));
      setVolumeOnChannel (Dcc.getCV(124),soundset_channel[5]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[5]);  //  play clip  4
      ss5delay=millis()+(long(Dcc.getCV(126)*MasterTimeConstant)); // Wait5
      ss5[4]=0;  ss5[5]=1;
    }
  if ((ss5[5]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(128));
      setVolumeOnChannel (Dcc.getCV(127),soundset_channel[5]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[5]);  //  play clip  5
      ss5[5]=0;    playing_sound_set[5]=false;
    }
  
//    ==========================   Sound Set 6 Begin Play
  if (ss6[0]==1) {
     ss6delay=millis()+(long(Dcc.getCV(130)*MasterTimeConstant)); // Wait1
     ss6[0]=0;  ss6[1]=1;
   soundset_channel[6]=Dcc.getCV(145); //load the channel in case it updated
    }
  if ((ss6[1]==1)&&(ss6delay<=millis())) {
    ttemp=(Dcc.getCV(132));
    setVolumeOnChannel (Dcc.getCV(131),soundset_channel[6]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[6]);  //  play clip 1
    ss6delay=millis()+(long(Dcc.getCV(133)*MasterTimeConstant)); // Wait2
    ss6[1]=0;  ss6[2]=1;
    }
  if ((ss6[2]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(135));
      setVolumeOnChannel (Dcc.getCV(134),soundset_channel[6]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[6]);  //  play clip 2
      ss6delay=millis()+(long(Dcc.getCV(136)*MasterTimeConstant)); // Wait3
      ss6[2]=0;  ss6[3]=1;
    }
  if ((ss6[3]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(138));
      setVolumeOnChannel (Dcc.getCV(137),soundset_channel[6]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[6]);  //  play clip 3
      ss6delay=millis()+(long(Dcc.getCV(139)*MasterTimeConstant)); // Wait4
      ss6[3]=0;  ss6[4]=1;
    }
  if ((ss6[4]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(141));
      setVolumeOnChannel (Dcc.getCV(140),soundset_channel[6]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[6]);  //  play clip  4
      ss6delay=millis()+(long(Dcc.getCV(142)*MasterTimeConstant)); // Wait5
      ss6[4]=0;  ss6[5]=1;
    }
  if ((ss6[5]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(144));
      setVolumeOnChannel (Dcc.getCV(143),soundset_channel[6]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[6]);  //  play clip  5
      ss6[5]=0;    playing_sound_set[6]=false;
    }
  
//    ==========================   Sound Set 7 Begin Play
  if (ss7[0]==1) {
     ss7delay=millis()+(long(Dcc.getCV(146)*MasterTimeConstant)); // Wait1
     ss7[0]=0;  ss7[1]=1;
   soundset_channel[7]=Dcc.getCV(161); //load the channel in case it updated
    }
  if ((ss7[1]==1)&&(ss7delay<=millis())) {
    ttemp=(Dcc.getCV(148));
    setVolumeOnChannel (Dcc.getCV(147),soundset_channel[7]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[7]);  //  play clip 1
    ss7delay=millis()+(long(Dcc.getCV(149)*MasterTimeConstant)); // Wait2
    ss7[1]=0;  ss7[2]=1;
    }
  if ((ss7[2]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(151));
      setVolumeOnChannel (Dcc.getCV(150),soundset_channel[7]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[7]);  //  play clip 2
      ss7delay=millis()+(long(Dcc.getCV(152)*MasterTimeConstant)); // Wait3
      ss7[2]=0;  ss7[3]=1;
    }
  if ((ss7[3]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(154));
      setVolumeOnChannel (Dcc.getCV(153),soundset_channel[7]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[7]);  //  play clip 3
      ss7delay=millis()+(long(Dcc.getCV(155)*MasterTimeConstant)); // Wait4
      ss7[3]=0;  ss7[4]=1;
    }
  if ((ss7[4]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(157));
      setVolumeOnChannel (Dcc.getCV(156),soundset_channel[7]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[7]);  //  play clip  4
      ss7delay=millis()+(long(Dcc.getCV(158)*MasterTimeConstant)); // Wait5
      ss7[4]=0;  ss7[5]=1;
    }
  if ((ss7[5]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(160));
      setVolumeOnChannel (Dcc.getCV(159),soundset_channel[7]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[7]);  //  play clip  5
      ss7[5]=0;    playing_sound_set[7]=false;
    }
//    ==========================   Sound Set 8 Begin Play
  if (ss8[0]==1) {
     ss8delay=millis()+(long(Dcc.getCV(162)*MasterTimeConstant)); // Wait1
     ss8[0]=0;  ss8[1]=1;
   soundset_channel[8]=Dcc.getCV(177); //load the channel in case it updated
    }
  if ((ss8[1]==1)&&(ss8delay<=millis())) {
    ttemp=(Dcc.getCV(164));
    setVolumeOnChannel (Dcc.getCV(163),soundset_channel[8]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[8]);  //  play clip 1
    ss8delay=millis()+(long(Dcc.getCV(165)*MasterTimeConstant)); // Wait2
    ss8[1]=0;  ss8[2]=1;
    }
  if ((ss8[2]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(167));
      setVolumeOnChannel (Dcc.getCV(166),soundset_channel[8]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[8]);  //  play clip 2
      ss8delay=millis()+(long(Dcc.getCV(168)*MasterTimeConstant)); // Wait3
      ss8[2]=0;  ss8[3]=1;
    }
  if ((ss8[3]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(170));
      setVolumeOnChannel (Dcc.getCV(169),soundset_channel[8]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[8]);  //  play clip 3
      ss8delay=millis()+(long(Dcc.getCV(171)*MasterTimeConstant)); // Wait4
      ss8[3]=0;  ss8[4]=1;
    }
  if ((ss8[4]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(173));
      setVolumeOnChannel (Dcc.getCV(172),soundset_channel[8]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[8]);  //  play clip  4
      ss8delay=millis()+(long(Dcc.getCV(174)*MasterTimeConstant)); // Wait5
      ss8[4]=0;  ss8[5]=1;
    }
  if ((ss8[5]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(176));
      setVolumeOnChannel (Dcc.getCV(175),soundset_channel[8]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[8]);  //  play clip  5
      ss8[5]=0;    playing_sound_set[8]=false;
    }
//    ==========================   Sound Set 9 Begin Play
  if (ss9[0]==1) {
     ss9delay=millis()+(long(Dcc.getCV(178)*MasterTimeConstant)); // Wait1
     ss9[0]=0;  ss9[1]=1;
   soundset_channel[9]=Dcc.getCV(193); //load the channel in case it updated
    }
  if ((ss9[1]==1)&&(ss9delay<=millis())) {
    ttemp=(Dcc.getCV(180));
    setVolumeOnChannel (Dcc.getCV(179),soundset_channel[9]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[9]);  //  play clip 1
    ss9delay=millis()+(long(Dcc.getCV(181)*MasterTimeConstant)); // Wait2
    ss9[1]=0;  ss9[2]=1;
    }
  if ((ss9[2]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(183));
      setVolumeOnChannel (Dcc.getCV(182),soundset_channel[9]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[9]);  //  play clip 2
      ss9delay=millis()+(long(Dcc.getCV(184)*MasterTimeConstant)); // Wait3
      ss9[2]=0;  ss9[3]=1;
    }
  if ((ss9[3]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(186));
      setVolumeOnChannel (Dcc.getCV(185),soundset_channel[9]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[9]);  //  play clip 3
      ss9delay=millis()+(long(Dcc.getCV(187)*MasterTimeConstant)); // Wait4
      ss9[3]=0;  ss9[4]=1;
    }
  if ((ss9[4]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(189));
      setVolumeOnChannel (Dcc.getCV(188),soundset_channel[9]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[9]);  //  play clip  4
      ss9delay=millis()+(long(Dcc.getCV(190)*MasterTimeConstant)); // Wait5
      ss9[4]=0;  ss9[5]=1;
    }
  if ((ss9[5]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(192));
      setVolumeOnChannel (Dcc.getCV(191),soundset_channel[9]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[9]);  //  play clip  5
      ss9[5]=0;    playing_sound_set[9]=false;
    }
//    ==========================   Sound Set 10 Begin Play
  if (ss10[0]==1) {
     ss10delay=millis()+(long(Dcc.getCV(194)*MasterTimeConstant)); // Wait1
     ss10[0]=0;  ss10[1]=1;
   soundset_channel[10]=Dcc.getCV(209); //load the channel in case it updated
    }
  if ((ss10[1]==1)&&(ss10delay<=millis())) {
    ttemp=(Dcc.getCV(196));
    setVolumeOnChannel (Dcc.getCV(195),soundset_channel[10]);
    if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[10]);  //  play clip 1
    ss10delay=millis()+(long(Dcc.getCV(197)*MasterTimeConstant)); // Wait2
    ss10[1]=0;  ss10[2]=1;
    }
  if ((ss10[2]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(199));
      setVolumeOnChannel (Dcc.getCV(198),soundset_channel[10]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[10]);  //  play clip 2
      ss10delay=millis()+(long(Dcc.getCV(200)*MasterTimeConstant)); // Wait3
      ss10[2]=0;  ss10[3]=1;
    }
  if ((ss10[3]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(202));
      setVolumeOnChannel (Dcc.getCV(201),soundset_channel[10]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[10]);  //  play clip 3
      ss10delay=millis()+(long(Dcc.getCV(203)*MasterTimeConstant)); // Wait4
      ss10[3]=0;  ss10[4]=1;
    }
  if ((ss10[4]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(205));
      setVolumeOnChannel (Dcc.getCV(204),soundset_channel[10]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[10]);  //  play clip  4
      ss10delay=millis()+(long(Dcc.getCV(206)*MasterTimeConstant)); // Wait5
      ss10[4]=0;  ss10[5]=1;
    }
  if ((ss10[5]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(208));
      setVolumeOnChannel (Dcc.getCV(207),soundset_channel[10]);
      if (ttemp!=0) playTrackOnChannel(ttemp,soundset_channel[10]);  //  play clip  5
      ss10[5]=0;    playing_sound_set[10]=false;
    }
}       // end  loop() 

void playTrackOnChannel ( byte dtrack, byte dchannel)  {
  switch ( dchannel )  {
    case 0:
      //DFSerial1.listen();
      if (dtrack!=255) {Player1.play(dtrack); delay(audiocmddelay); }
	    else {Player1.play(random(First_Track,Last_Track+1)); delay(audiocmddelay); }
    break;
    case 1:
    default:
      //DFSerial2.listen();
      if (dtrack!=255) {Player2.play(dtrack); delay(audiocmddelay); }
	    else {Player2.play(random(First_Track,Last_Track+1)); delay(audiocmddelay); }
    break;
  }
}
void setVolumeOnChannel ( byte dvolume, byte dchannel)  {
  if(dvolume>30) return;    // Don't change the volume if out of range
  switch ( dchannel )  {
    case 0:
	  Player1.volume (dvolume);
          delay(audiocmddelay);
    break;
    case 1:
    default:
      Player2.volume (dvolume);
          delay(audiocmddelay);
    break;
  }
}
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
#ifdef DEBUG
   //Serial.print("Addr= ");
   //Serial.println(Addr, DEC) ;
   //Serial.print("FuncState= ");
   //Serial.println(FuncState, DEC) ;
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
    exec_function( 7, FunctionPin7, (FuncState & FN_BIT_07)>>2 );
    exec_function( 8, FunctionPin8, (FuncState & FN_BIT_08)>>3 );
    break;
    
  case FN_9_12:
    exec_function( 9, FunctionPin9,   (FuncState & FN_BIT_09));
    exec_function( 10, FunctionPin10, (FuncState & FN_BIT_10)>>1 );
    exec_function( 11, FunctionPin11, (FuncState & FN_BIT_11)>>2 );
    exec_function( 12, FunctionPin12, (FuncState & FN_BIT_12)>>3 );
    break;
  case FN_13_20:   //Function Group 2 FuncState == F20-F13 Function Control
    exec_function( 13, FunctionPin13, (FuncState & FN_BIT_13));
    exec_function( 14, FunctionPin14, (FuncState & FN_BIT_14)>>1 );
    //exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
    //exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
      break;
  
  case FN_21_28:
      break;  
  }
}       // end  notifyDccFunc
void exec_function (int function, int pin, int FuncState)  {
#ifdef DEBUG
   //Serial.print("ex function= ");
   //Serial.println(function, DEC) ;
   //Serial.print("FuncState= ");
   //Serial.println(FuncState, DEC) ;
#endif
  switch ( Dcc.getCV( 30+function) )  {  // Config 
    case 0:    // Master Disable
      Function0_value = byte(FuncState);
      break;
    case 1:   //   play sound set [ function ]
        
    case 2:   //   play sound set [ function ]
             
    case 3:   //   play sound set [ function ]
        
    case 4:   //   play sound set [ function ]
        
    case 5:   //   play sound set [ function ]
        
    case 6:   //   play sound set [ function ]
        
    case 7:   //   play sound set [ function ]
        
    case 8:   //   play sound set [ function ]
        
    case 9:   //   play sound set [ function ]
        
    case 10:  //   play sound set [ function ]
      function_value[function] = byte(FuncState);
      break;
    case 11:  // On - Off LED
      if (MasterDecoderDisable == 0)  digitalWrite (pin, FuncState);
        else digitalWrite (pin, 0);  //  Master Disable is ON so force LEDs off
      break;
    default:
      break;;
  }
}      // end exec_function

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
