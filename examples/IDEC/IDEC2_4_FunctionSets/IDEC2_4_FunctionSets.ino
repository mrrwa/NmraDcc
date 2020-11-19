// Interactive Decoder Random Switches   IDEC2_4_FunctionSetsDev.ino
// Version 1.08 Geoff Bunza 2020
// Works with both short and long DCC Addesses
// This decoder will control Switch Sequences, servos, sounds and LEDs
// F0=Master Function OFF = Function ON DISABLES the decoder
/* F0 is configured as Master Decoder Disable Override  ON==Disable Decoder
 * Input Pin for Decoder Disable  Pin 16/A2 Active LOW
// F1-F8 Eight Switch Sets 1-8 controlled by Input pins 3,4,5,6,7,8,9,10  respectively
 *  F1-F10  also runs switch sets (1-8)   All Switch Sets are defined by groups of 16 CVs
  - Either a DCC Function 1-10 on OR an Input Pin (3,4,5,6,7,8,9,10) Switched Low enables a decoder function (ON)
  - BOTH the respective DCC Decoder Function 1-8 must be Off AND its respective Input Pin  (3,4,5,6,7,8,9,10) 
    MUST be High for a decoder function to be considered disabled
  - A decoder function LEFT ENABLED will repreat the respecpective action as long as it is enabled
 * Switch Set CV's are 5 groups of 3 CVs each:  
       CV1 - A delay (0-255) which will be multiplied by the 
	   MasterTimeConstant setting time increments from milliseconds to minutes
	   0 = No Delay
       CV2 - A Mode or Command byte Describing what will be executed in this Switch Step, including:
	  0 = No Operation / Null /Skip
	  1 = Simple pin switch on/off
	  2= Random pin switch on/off
	  3 = Weighted Random pin switch on/off  default is 60% ON time but can be set to anything 1-99%
	  4 = Play sound track  using fpin value for the track 1-126, 0 = Skip Play, 127 = Select Random Track
	        from First_Track to Last_Track inclusive; 
	        MSB=0->No Volume Change MSB=1 -> Set Volume to default_volume
	  5 = Position Servo to 0-180 full speed of travel
	  6 = Dual pin on/off used for alternate blink fpin and fpin+1 (MSB set value for fpin state)
	  7 = Start another Switching set based on the fpin argument (Used to chain Switch Sets)
	  8 = Start another Switching set based on the fpin argument ONLY if NOT already started
       CV3 - An argument representing the Pin number affected in the lower 7 bits and the High bit (0x80 or 128) a value
	   or a general parameter like a servo position, a Sound track, or a sound set number to jump to
 * Switch sets start with CVs:  50,66,82,98,114,130,146,162,178,194
 * MAX one of 11 Configurations per pin function:
 *  Config 0=DISABLE On/Off,1-10=Switch Control 1-8
 
PRO MINI PIN ASSIGNMENT:
2 - DCC Input
3 - Input Pin Switch   1
4 - Input Pin Switch   2
5 - Input Pin Switch   3
6 - Input Pin Switch   4
7 - Input Pin Switch   5
8 - Input Pin Switch   6
9 - Input Pin Switch   7
10 - Input Pin Switch  8
11 - Switch 1
12 - Switch 2
13 - Switch 3
14 A0 - Switch 4  or  default_servo_pin 
15 A1 - Switch 5  or default sound player pin (TX) connected to  DFPlayer1 (RX) Pin 2 via 1K Ohm 1/4W Resistor
16 A2 - Input Pin for MasterDecoderDisable Active LOW
17 A3 - Switch 6
18 A4 - Switch 7
19 A5 - Switch 8
*/
// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** REMOVE THE "//" IN THE FOLOWING LINE TO enable sound  DFPlayer on Pin 15
//#define SOUND_PLAYER15

// ******** REMOVE THE "//" IN THE FOLOWING LINE TO enable SERVO USE
//#define USE_SERVO14

// ******** REMOVE THE "//" IN THE FOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR  *****NOTE Turning DEBUG ON changes ALL Timing!!
//#define DEBUG


#include <NmraDcc.h>

#ifdef USE_SERVO14
#include <SoftwareServo.h>
SoftwareServo servo[2];
#endif
#define default_servo_pin  14

#ifdef SOUND_PLAYER15
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
SoftwareSerial DFSerial1(22,15); // PRO MINI RX, PRO MINI TX serial to DFPlayer
DFRobotDFPlayerMini Player1;
#endif
#define  default_volume  25    //  sets default volume 0-30,  0 == OFF,  >30 == Skip Track
#define First_Track 1   // Play Random Tracks First_Track#=Start_Track  >=1
#define Last_Track 12   // Play Random Tracks Last_Track= Last Playable Track in Range  <= Last Numbered Track
const int audiocmddelay = 34;

#define This_Decoder_Address 24
uint8_t CV_DECODER_MASTER_RESET = 252;

//Uncomment ONLY ONE of the following:
//#define  MasterTimeConstant  10L      // 10's of milliseconds Timing
#define  MasterTimeConstant  100L     // Tenths of a second Timing
//#define  MasterTimeConstant  1000L    // Seconds Timing
//#define  MasterTimeConstant  10000L   // 10's of Seconds Timing
//#define  MasterTimeConstant  60000L   // Minutes Timing
//#define  MasterTimeConstant  3600000L // Hours Timing

int del_tim = 4000; 
uint16_t ttemp, i;

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
bool run_switch_set [ ] = {false,false,false,false,false,false,false,false,false,false,false};
byte switchset_channel[ ]={0,0,0,0,0,0,0,0,0,0,0};
const int MasterDecoderDisablePin = 16; // D16/A0  Master Decoder Disable Input Pin Active LOW

const int numINpins = 9;              // Number of INput pins to initialize
byte inputpins [] = {3,4,5,6,7,8,9,10,16};  //These are all the Input Pins
const int numfpins = 8;              // Number of Output pins to initialize
const int num_active_functions = 11;   // Number of Functions stating with F0
byte fpins [] = {11,12,13,14,15,17,18,19};  //These are all the Output Pins (first 15 is placeholder)
const int FunctionPin0 = 20;    // D14/A0 DFPlayer Transmit (TX)  Pin 3
const int FunctionPin1 = 20;    // A2 LED Place holders ONLY
const int FunctionPin2 = 20;    // A3 LED Place holders ONLY
const int FunctionPin3 = 20;    // A4 LED Place holders ONLY
const int FunctionPin4 = 20;    // A5 LED Place holders ONLY
const int FunctionPin5  = 20;   // A6 LED Place holders ONLY
const int FunctionPin6  = 20;   // A7 Place holders ONLY
const int FunctionPin7  = 20;   // A8 Place holders ONLY
const int FunctionPin8  = 20;   // A9 Place holders ONLY
#define switch1 11
#define switch2 12
#define switch3 13
#define switch4 14
#define switch5 15
#define switch6 17
#define switch7 18
#define switch8 19
#define on 0x80
#define off 0
const int FunctionPin9  = 20;   // Place holders ONLY
const int FunctionPin10 = 20;   // Place holders ONLY
const int FunctionPin11 = 20;   // F13 LED
const int FunctionPin12 = 20;   // F12 LED
const int FunctionPin13 = 20;   // F13 LED
const int FunctionPin14 = 20;   // F14 LED
const int FunctionPin15 = 20;   // Place holders ONLY
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

  {30, 0}, //F0 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  
  {31, 1}, //F1 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {32, 2}, //F2 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {33, 3}, //F3 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {34, 4}, //F4 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {35, 5}, //F5 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {36, 6}, //F6 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {37, 7}, //F7 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {38, 8}, //F8 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {39, 9}, //F9 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  {40, 10}, //F10 Config 0=DISABLE On/Off,1-8=Switch Control 1-10,11=LED On/Off
  
  {41, 11}, 
  {42, 11}, 
  {43, 11}, 
  {44, 11}, 
  {45, 22}, //F15 not used
  
  {50, 3}, // Wait1 0-254 0.1 Seconds  // switch SET 1
  {51, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {52,  switch3+on}, // Switch Pin1
  {53,  6}, // Wait2 0-254 0.1 Seconds
  {54, 5},  // Switch Mode
  {55, 165}, // Switch Pin2
  {56,  10}, // Wait3 0-254 0.1 Seconds
  {57, 4}, // Switch Mode
  {58,  11+128}, // Switch Pin3
  {59, 10},  // Wait4 0-254 0.1 Seconds
  {60, 5}, // Switch Mode
  {61, 20}, // Switch Pin4
  {62, 15}, // Wait5 0-254 0.1 Seconds
  {63, 1}, // Switch Mode
  {64, switch3+off},  // Switch Pin5
  {65, 0}, // switch Set Channel == LSB 0/1
  
  {66, 2}, // Wait1 0-254 0.1 Seconds  // switch SET 2
  {67, 2}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {68, switch3+on}, // Switch Pin1
  {69, 6}, // Wait2 0-254 0.1 Seconds
  {70, 2},  // Switch Mode
  {71, switch3+off}, // Switch Pin2
  {72, 6}, // Wait3 0-254 0.1 Seconds
  {73, 2}, // Switch Mode
  {74, switch3+on}, // Switch Pin3
  {75, 6},  // Wait4 0-254 0.1 Seconds
  {76, 2}, // Switch Mode
  {77, switch3+off}, // Switch Pin4
  {78, 8}, // Wait5 0-254 0.1 Seconds
  {79, 8}, // Switch Mode
  {80, 3},  // Switch Pin5
  {81, 0}, // switch Set Channel == LSB 0/1
  
  {82, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 3
  {83, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {84, switch3+on}, // Switch Pin1
  {85, 8}, // Wait2 0-254 0.1 Seconds
  {86, 1},  // Switch Mode
  {87, switch3+off}, // Switch Pin2
  {88, 8}, // Wait3 0-254 0.1 Seconds
  {89, 1}, // Switch Mode
  {90, switch3+on}, // Switch Pin3
  {91, 8},  // Wait4 0-254 0.1 Seconds
  {92, 1}, // Switch Mode
  {93, switch3+off}, // Switch Pin4
  {94, 8}, // Wait5 0-254 0.1 Seconds
  {95, 8}, // Switch Mode
  {96, 4},  // Switch Pin5
  {97, 0}, // switch Set Channel == LSB 0/1

  {98, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 4
  {99, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {100, switch3+on}, // Switch Pin1
  {101, 8}, // Wait2 0-254 0.1 Seconds
  {102, 1},  // Switch Mode
  {103, switch3+off}, // Switch Pin2
  {104, 8}, // Wait3 0-254 0.1 Seconds
  {105, 1}, // Switch Mode
  {106, switch3+on}, // Switch Pin3
  {107, 8},  // Wait4 0-254 0.1 Seconds
  {108, 1}, // Switch Mode
  {109, switch3+off}, // Switch Pin4
  {110, 8}, // Wait5 0-254 0.1 Seconds
  {111, 8}, // Switch Mode
  {112, 5},  // Switch Pin5
  {113, 0}, // switch Set Channel == LSB 0/1
  
  {114, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 5
  {115, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {116, switch3+on}, // Switch Pin1
  {117, 8}, // Wait2 0-254 0.1 Seconds
  {118, 1},  // Switch Mode
  {119, switch3+off}, // Switch Pin2
  {120, 8}, // Wait3 0-254 0.1 Seconds
  {121, 1}, // Switch Mode
  {122, switch3+on}, // Switch Pin3
  {123, 8},  // Wait4 0-254 0.1 Seconds
  {124, 1}, // Switch Mode
  {125, switch3+off}, // Switch Pin4
  {126, 8}, // Wait5 0-254 0.1 Seconds
  {127, 8}, // Switch Mode
  {128, 6},  // Switch Pin5
  {129, 0}, // switch Set Channel == LSB 0/1
  
  {130, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 6
  {131, 6}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {132, switch3+on}, // Switch Pin1
  {133, 9}, // Wait2 0-254 0.1 Seconds
  {134, 6},  // Switch Mode
  {135, switch3+off}, // Switch Pin2
  {136, 9}, // Wait3 0-254 0.1 Seconds
  {137, 6}, // Switch Mode
  {138, switch3+on}, // Switch Pin3
  {139, 9},  // Wait4 0-254 0.1 Seconds
  {140, 6}, // Switch Mode
  {141, switch3+off}, // Switch Pin4
  {142, 9}, // Wait5 0-254 0.1 Seconds
  {143, 8}, // Switch Mode
  {144, 7},  // Switch Pin5
  {145, 0}, // switch Set Channel == LSB 0/1
  
  {146, 0}, // Wait1 0-254 0.1 Seconds // switch SET 7
  {147, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {148, switch3+on}, // Switch Pin1
  {149, 8}, // Wait2 0-254 0.1 Seconds
  {150, 1},  // Switch Mode
  {151, switch3+off}, // Switch Pin2
  {152, 8}, // Wait3 0-254 0.1 Seconds
  {153, 1}, // Switch Mode
  {154, switch3+on}, // Switch Pin3
  {155, 8},  // Wait4 0-254 0.1 Seconds
  {156, 1}, // Switch Mode
  {157, switch3+off}, // Switch Pin4
  {158, 8}, // Wait5 0-254 0.1 Seconds
  {159, 8}, // Switch Mode
  {160, 8},  // Switch Pin5
  {161, 0}, // switch Set Channel == LSB 0/1
  
  {162, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 8
  {163, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {164, switch3+on}, // Switch Pin1
  {165, 8}, // Wait2 0-254 0.1 Seconds
  {166, 1},  // Switch Mode
  {167, switch3+off}, // Switch Pin2
  {168, 8}, // Wait3 0-254 0.1 Seconds
  {169, 1}, // Switch Mode
  {170, switch3+on}, // Switch Pin3
  {171, 8},  // Wait4 0-254 0.1 Seconds
  {172, 8}, // Switch Mode
  {173, switch3+off}, // Switch Pin4
  {174, 8}, // Wait5 0-254 0.1 Seconds
  {175, 8}, // Switch Mode
  {176, 9},  // Switch Pin5
  {177, 0}, // switch Set Channel == LSB 0/1
  
  {178, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 9
  {179, 1}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {180, switch3+on}, // Switch Pin1
  {181, 9}, // Wait2 0-254 0.1 Seconds
  {182, 1},  // Switch Mode
  {183, switch3+off}, // Switch Pin2
  {184, 8}, // Wait3 0-254 0.1 Seconds
  {185, 1}, // Switch Mode
  {186, switch2+on}, // Switch Pin3
  {187, 12},  // Wait4 0-254 0.1 Seconds
  {188, 1}, // Switch Mode
  {189, switch2+off}, // Switch Pin4
  {190, 0}, // Wait5 0-254 0.1 Seconds
  {191, 0}, // Switch Mode
  {192, 0},  // Switch Pin5
  {193, 0}, // switch Set Channel == LSB 0/1
  
  {194, 4}, // Wait1 0-254 0.1 Seconds  // switch SET 10
  {195, 10}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {196, switch3+on}, // Switch Pin1
  {197, 10}, // Wait2 0-254 0.1 Seconds
  {198, 10},  // Switch Mode
  {199, switch3+on}, // Switch Pin2
  {200, 4}, // Wait3 0-254 0.1 Seconds
  {201, 10}, // Switch Mode
  {202, switch3+on}, // Switch Pin3
  {203, 4},  // Wait4 0-254 0.1 Seconds
  {204, 10}, // Switch Mode
  {205, switch3+on}, // Switch Pin4
  {206, 6}, // Wait5 0-254 0.1 Seconds
  {207, 10}, // Switch Mode
  {208, switch3+off},  // Switch Pin5
  {209, 0}, // switch Set Channel == LSB 0/1

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

#ifdef SOUND_PLAYER15
  DFSerial1.begin (9600);
  Player1.begin (DFSerial1);
#endif

#ifdef USE_SERVO14
  servo[0].attach(default_servo_pin);  // Start Servo on default_servo_pin  //Position Servo
  delay(50);
  SoftwareServo::refresh();
#endif

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
    for (i=30; i<41; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=30; i<41; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 1");
    for (i=50; i<66; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=50; i<66; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 2");
    for (i=66; i<82; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=66; i<82; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 3");
    for (i=82; i<98; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=82; i<98; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 4");
    for (i=98; i<114; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=98; i<114; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 5");
    for (i=114; i<130; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=114; i<130; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 6");
    for (i=130; i<146; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=130; i<146; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 7");
    for (i=146; i<162; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=146; i<162; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 8");
    for (i=162; i<178; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=162; i<178; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 9");
    for (i=178; i<194; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=178; i<194; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Switch Set 10");
    for (i=194; i<210; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=194; i<210; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
#endif   
}
void loop()   //***********************************************************************************
{
  //MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
#ifdef USE_SERVO14
  SoftwareServo::refresh();
#endif

  //delay(1);
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
        if (((digitalRead(3)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
          ss1[0]=1; run_switch_set[cv_value]=true; 
#ifdef DEBUG
    Serial.print(" cv_value: ");
    Serial.println(cv_value, DEC) ;
    Serial.print(" function_value[cv_value]: ");
    Serial.println(function_value[cv_value], DEC) ;
    Serial.print(" ss1[0]: ");
    Serial.println(ss1[0], DEC) ;
    Serial.print(" run_switch_set[cv_value]: ");
    Serial.println(run_switch_set[cv_value], DEC) ;
#endif
          }
      break;
      case 2:   //   
         if (((digitalRead(4)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss2[0]=1; run_switch_set[cv_value]=true; 
           }
      break;
      case 3:   // 
          if (((digitalRead(5)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss3[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 4:   // 
          if (((digitalRead(6)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss4[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 5:   // 
          if (((digitalRead(7)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss5[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 6:   // 
          if (((digitalRead(8)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
          ss6[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 7:   // 
          if (((digitalRead(9)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss7[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 8:   // 
          if (((digitalRead(10)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss8[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 9:   // 
          if ((function_value[cv_value]==1) && !run_switch_set[cv_value]) {
           ss9[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 10:   // 
          if ((function_value[cv_value]==1) && !run_switch_set[cv_value]) {
          ss10[0]=1; run_switch_set[cv_value]=true; }
      break;
      case 11:  //  Extra
      default:
      break;
		}
      }
    }
   //    ==========================   switch Set 1 Start Run
  if (ss1[0]==1) {
     ss1delay=millis()+(long(Dcc.getCV(50)*MasterTimeConstant)); // Wait1
     ss1[0]=0;  ss1[1]=1;  
    }
  if ((ss1[1]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(52));
#ifdef DEBUG
    Serial.print(" Here 1: ");
    Serial.println(ttemp, DEC) ;
#endif
    if (ttemp!=0) exec_switch_function(Dcc.getCV(51),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(53)*MasterTimeConstant)); // Wait2
    ss1[1]=0;  ss1[2]=1;
    }
  if ((ss1[2]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(55));
#ifdef DEBUG
    Serial.print(" Here 2: ");
    Serial.println(ttemp, DEC) ;
#endif
      if (ttemp!=0) exec_switch_function (Dcc.getCV(54),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(56)*MasterTimeConstant)); // Wait3
      ss1[2]=0;  ss1[3]=1;
    }
  if ((ss1[3]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(58));
#ifdef DEBUG
    Serial.print(" Here 3: ");
    Serial.println(ttemp, DEC) ;
#endif
      if (ttemp!=0) exec_switch_function(Dcc.getCV(57),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(59)*MasterTimeConstant)); // Wait4
      ss1[3]=0;  ss1[4]=1;
    }
  if ((ss1[4]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(61));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(60),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(62)*MasterTimeConstant)); // Wait5
      ss1[4]=0;  ss1[5]=1;
    }
  if ((ss1[5]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(64));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(63),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss1[5]=0;    run_switch_set[1]=false;
    }
//    ==========================   switch Set 2 Start Run
  if (ss2[0]==1) {
     ss2delay=millis()+(long(Dcc.getCV(66)*MasterTimeConstant)); // Wait1
     ss2[0]=0;  ss2[1]=1;
    }
  if ((ss2[1]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(68));
#ifdef DEBUG
    Serial.print(" Here 21: ");
    Serial.println(ttemp, DEC) ;
#endif
    if (ttemp!=0) exec_switch_function(Dcc.getCV(67),ttemp&0x3f&0x3f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(69)*MasterTimeConstant)); // Wait2
    ss2[1]=0;  ss2[2]=1;
    }
  if ((ss2[2]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(71));
#ifdef DEBUG
    Serial.print(" Here 22: ");
    Serial.println(ttemp, DEC) ;
#endif
      if (ttemp!=0) exec_switch_function(Dcc.getCV(70),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(72)*MasterTimeConstant)); // Wait3
      ss2[2]=0;  ss2[3]=1;
    }
  if ((ss2[3]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(74));
#ifdef DEBUG
    Serial.print(" Here 23: ");
    Serial.println(ttemp, DEC) ;
#endif
      if (ttemp!=0) exec_switch_function(Dcc.getCV(73),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(75)*MasterTimeConstant)); // Wait4
      ss2[3]=0;  ss2[4]=1;
    }
  if ((ss2[4]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(77));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(76),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(78)*MasterTimeConstant)); // Wait5
      ss2[4]=0;  ss2[5]=1;
    }
  if ((ss2[5]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(80));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(79),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss2[5]=0;    run_switch_set[2]=false;
    }
//    ==========================   switch Set 3 Start Run
  if (ss3[0]==1) {
     ss3delay=millis()+(long(Dcc.getCV(82)*MasterTimeConstant)); // Wait1
     ss3[0]=0;  ss3[1]=1;
    }
  if ((ss3[1]==1)&&(ss3delay<=millis())) {
    ttemp=(Dcc.getCV(84));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(83),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss3delay=millis()+(long(Dcc.getCV(85)*MasterTimeConstant)); // Wait2
    ss3[1]=0;  ss3[2]=1;
    }
  if ((ss3[2]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(87));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(86),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss3delay=millis()+(long(Dcc.getCV(88)*MasterTimeConstant)); // Wait3
      ss3[2]=0;  ss3[3]=1;
    }
  if ((ss3[3]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(90));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(89),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss3delay=millis()+(long(Dcc.getCV(91)*MasterTimeConstant)); // Wait4
      ss3[3]=0;  ss3[4]=1;
    }
  if ((ss3[4]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(93));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(92),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss3delay=millis()+(long(Dcc.getCV(94)*MasterTimeConstant)); // Wait5
      ss3[4]=0;  ss3[5]=1;
    }
  if ((ss3[5]==1)&&(ss3delay<=millis())) {
      ttemp=(Dcc.getCV(96));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(95),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss3[5]=0;    run_switch_set[3]=false;
    }
//    ==========================   switch Set 4 Start Run
  if (ss4[0]==1) {
     ss4delay=millis()+(long(Dcc.getCV(98)*MasterTimeConstant)); // Wait1
     ss4[0]=0;  ss4[1]=1;
    }
  if ((ss4[1]==1)&&(ss4delay<=millis())) {
    ttemp=(Dcc.getCV(100));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(99),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss4delay=millis()+(long(Dcc.getCV(101)*MasterTimeConstant)); // Wait2
    ss4[1]=0;  ss4[2]=1;
    }
  if ((ss4[2]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(103));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(102),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss4delay=millis()+(long(Dcc.getCV(104)*MasterTimeConstant)); // Wait3
      ss4[2]=0;  ss4[3]=1;
    }
  if ((ss4[3]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(106));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(105),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss4delay=millis()+(long(Dcc.getCV(107)*MasterTimeConstant)); // Wait4
      ss4[3]=0;  ss4[4]=1;
    }
  if ((ss4[4]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(109));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(108),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss4delay=millis()+(long(Dcc.getCV(110)*MasterTimeConstant)); // Wait5
      ss4[4]=0;  ss4[5]=1;
    }
  if ((ss4[5]==1)&&(ss4delay<=millis())) {
      ttemp=(Dcc.getCV(112));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(111),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss4[5]=0;    run_switch_set[4]=false;
    }
//    ==========================   switch Set 5 Start Run
  if (ss5[0]==1) {
     ss5delay=millis()+(long(Dcc.getCV(114)*MasterTimeConstant)); // Wait1
     ss5[0]=0;  ss5[1]=1;
    }
  if ((ss5[1]==1)&&(ss5delay<=millis())) {
    ttemp=(Dcc.getCV(116));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(115),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss5delay=millis()+(long(Dcc.getCV(117)*MasterTimeConstant)); // Wait2
    ss5[1]=0;  ss5[2]=1;
    }
  if ((ss5[2]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(119));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(118),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss5delay=millis()+(long(Dcc.getCV(120)*MasterTimeConstant)); // Wait3
      ss5[2]=0;  ss5[3]=1;
    }
  if ((ss5[3]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(122));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(121),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss5delay=millis()+(long(Dcc.getCV(123)*MasterTimeConstant)); // Wait4
      ss5[3]=0;  ss5[4]=1;
    }
  if ((ss5[4]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(125));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(124),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss5delay=millis()+(long(Dcc.getCV(126)*MasterTimeConstant)); // Wait5
      ss5[4]=0;  ss5[5]=1;
    }
  if ((ss5[5]==1)&&(ss5delay<=millis())) {
      ttemp=(Dcc.getCV(128));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(127),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss5[5]=0;    run_switch_set[5]=false;
    }
  
//    ==========================   switch Set 6 Start Run
  if (ss6[0]==1) {
     ss6delay=millis()+(long(Dcc.getCV(130)*MasterTimeConstant)); // Wait1
     ss6[0]=0;  ss6[1]=1;
    }
  if ((ss6[1]==1)&&(ss6delay<=millis())) {
    ttemp=(Dcc.getCV(132));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(131),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss6delay=millis()+(long(Dcc.getCV(133)*MasterTimeConstant)); // Wait2
    ss6[1]=0;  ss6[2]=1;
    }
  if ((ss6[2]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(135));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(134),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss6delay=millis()+(long(Dcc.getCV(136)*MasterTimeConstant)); // Wait3
      ss6[2]=0;  ss6[3]=1;
    }
  if ((ss6[3]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(138));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(137),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss6delay=millis()+(long(Dcc.getCV(139)*MasterTimeConstant)); // Wait4
      ss6[3]=0;  ss6[4]=1;
    }
  if ((ss6[4]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(141));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(140),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss6delay=millis()+(long(Dcc.getCV(142)*MasterTimeConstant)); // Wait5
      ss6[4]=0;  ss6[5]=1;
    }
  if ((ss6[5]==1)&&(ss6delay<=millis())) {
      ttemp=(Dcc.getCV(144));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(143),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss6[5]=0;    run_switch_set[6]=false;
    }
  
//    ==========================   switch Set 7 Start Run
  if (ss7[0]==1) {
     ss7delay=millis()+(long(Dcc.getCV(146)*MasterTimeConstant)); // Wait1
     ss7[0]=0;  ss7[1]=1;
    }
  if ((ss7[1]==1)&&(ss7delay<=millis())) {
    ttemp=(Dcc.getCV(148));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(147),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss7delay=millis()+(long(Dcc.getCV(149)*MasterTimeConstant)); // Wait2
    ss7[1]=0;  ss7[2]=1;
    }
  if ((ss7[2]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(151));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(150),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss7delay=millis()+(long(Dcc.getCV(152)*MasterTimeConstant)); // Wait3
      ss7[2]=0;  ss7[3]=1;
    }
  if ((ss7[3]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(154));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(153),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss7delay=millis()+(long(Dcc.getCV(155)*MasterTimeConstant)); // Wait4
      ss7[3]=0;  ss7[4]=1;
    }
  if ((ss7[4]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(157));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(156),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss7delay=millis()+(long(Dcc.getCV(158)*MasterTimeConstant)); // Wait5
      ss7[4]=0;  ss7[5]=1;
    }
  if ((ss7[5]==1)&&(ss7delay<=millis())) {
      ttemp=(Dcc.getCV(160));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(159),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss7[5]=0;    run_switch_set[7]=false;
    }
//    ==========================   switch Set 8 Start Run
  if (ss8[0]==1) {
     ss8delay=millis()+(long(Dcc.getCV(162)*MasterTimeConstant)); // Wait1
     ss8[0]=0;  ss8[1]=1;
    }
  if ((ss8[1]==1)&&(ss8delay<=millis())) {
    ttemp=(Dcc.getCV(164));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(163),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss8delay=millis()+(long(Dcc.getCV(165)*MasterTimeConstant)); // Wait2
    ss8[1]=0;  ss8[2]=1;
    }
  if ((ss8[2]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(167));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(166),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss8delay=millis()+(long(Dcc.getCV(168)*MasterTimeConstant)); // Wait3
      ss8[2]=0;  ss8[3]=1;
    }
  if ((ss8[3]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(170));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(169),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss8delay=millis()+(long(Dcc.getCV(171)*MasterTimeConstant)); // Wait4
      ss8[3]=0;  ss8[4]=1;
    }
  if ((ss8[4]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(173));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(172),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss8delay=millis()+(long(Dcc.getCV(174)*MasterTimeConstant)); // Wait5
      ss8[4]=0;  ss8[5]=1;
    }
  if ((ss8[5]==1)&&(ss8delay<=millis())) {
      ttemp=(Dcc.getCV(176));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(175),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss8[5]=0;    run_switch_set[8]=false;
    }
//    ==========================   switch Set 9 Start Run
  if (ss9[0]==1) {
     ss9delay=millis()+(long(Dcc.getCV(178)*MasterTimeConstant)); // Wait1
     ss9[0]=0;  ss9[1]=1;
    }
  if ((ss9[1]==1)&&(ss9delay<=millis())) {
    ttemp=(Dcc.getCV(180));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(179),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss9delay=millis()+(long(Dcc.getCV(181)*MasterTimeConstant)); // Wait2
    ss9[1]=0;  ss9[2]=1;
    }
  if ((ss9[2]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(183));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(182),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss9delay=millis()+(long(Dcc.getCV(184)*MasterTimeConstant)); // Wait3
      ss9[2]=0;  ss9[3]=1;
    }
  if ((ss9[3]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(186));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(185),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss9delay=millis()+(long(Dcc.getCV(187)*MasterTimeConstant)); // Wait4
      ss9[3]=0;  ss9[4]=1;
    }
  if ((ss9[4]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(189));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(188),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss9delay=millis()+(long(Dcc.getCV(190)*MasterTimeConstant)); // Wait5
      ss9[4]=0;  ss9[5]=1;
    }
  if ((ss9[5]==1)&&(ss9delay<=millis())) {
      ttemp=(Dcc.getCV(192));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(191),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss9[5]=0;    run_switch_set[9]=false;
    }
//    ==========================   switch Set 10 Start Run
  if (ss10[0]==1) {
     ss10delay=millis()+(long(Dcc.getCV(194)*MasterTimeConstant)); // Wait1
     ss10[0]=0;  ss10[1]=1;
    }
  if ((ss10[1]==1)&&(ss10delay<=millis())) {
    ttemp=(Dcc.getCV(196));
    if (ttemp!=0) exec_switch_function(Dcc.getCV(195),ttemp&0x3f,ttemp>>7);  //  execute switch function 1
    ss10delay=millis()+(long(Dcc.getCV(197)*MasterTimeConstant)); // Wait2
    ss10[1]=0;  ss10[2]=1;
    }
  if ((ss10[2]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(199));
      //setVolumeOnChannel (Dcc.getCV(198));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(198),ttemp&0x3f,ttemp>>7);  //  execute switch function 2
      ss10delay=millis()+(long(Dcc.getCV(200)*MasterTimeConstant)); // Wait3
      ss10[2]=0;  ss10[3]=1;
    }
  if ((ss10[3]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(202));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(201),ttemp&0x3f,ttemp>>7);  //  execute switch function 3
      ss10delay=millis()+(long(Dcc.getCV(203)*MasterTimeConstant)); // Wait4
      ss10[3]=0;  ss10[4]=1;
    }
  if ((ss10[4]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(205));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(204),ttemp&0x3f,ttemp>>7);  //  execute switch function  4
      ss10delay=millis()+(long(Dcc.getCV(206)*MasterTimeConstant)); // Wait5
      ss10[4]=0;  ss10[5]=1;
    }
  if ((ss10[5]==1)&&(ss10delay<=millis())) {
      ttemp=(Dcc.getCV(208));
      if (ttemp!=0) exec_switch_function(Dcc.getCV(207),ttemp&0x3f,ttemp>>7);  //  execute switch function  5
      ss10[5]=0;    run_switch_set[10]=false;
    }
}       // end  loop() 

void exec_switch_function (byte switch_function, byte fpin,byte fbit)  {
  if (MasterDecoderDisable == 1) return;

  // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next 
	switch ( switch_function )  {  // find the switch function to execute 
    case 0:    //  0 == No function
	default:
	break;
	case 1:    //
	  digitalWrite ( fpin,fbit );  //  Simple pin switch on/off
	break;
	case 2:    // 
	  if (fbit!=0) digitalWrite ( fpin, random ( 0,2) );  //  Random pin switch on/off
	    else digitalWrite ( fpin, 0 );  //  Force the bit OFF
	break;
	case 3:    //
	  if (fbit!=0) digitalWrite ( fpin, Weighted_ON() );  //  Weighted Random pin switch on/off
	    else digitalWrite ( fpin, 0 );  //  Force the bit OFF
	break;
	case 4:    // 
#ifdef SOUND_PLAYER15
	  setVolumeOnChannel (default_volume * fbit); //  set volume level
	  playTrackOnChannel(fpin);                   //  play sound track  fpin 1-127
#endif
	break;
	case 5:    //
#ifdef USE_SERVO14
    servo[0].write(fpin|(fbit<<7));                // Position Servo
#endif
	break;
	case 6:    // 
	  digitalWrite ( fpin,fbit );  //  Dual pin on/off  alternate blink
	  digitalWrite ( fpin+1,(~fbit)&0x01 );  //  Dual pin on/off  alternate blink
	break;
	case 7:    //   Start up another switch set
	{
	  switch ( fpin )  {  // Start another Switching set based on the fpin argument
        case 0:    //  
		default:
		break;
        case 1:    //   Start Switch Set 1
		  ss1[0] = 1;  run_switch_set[fpin]=true;
		break;
        case 2:    //    Start Switch Set 2
		  ss2[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 3:    //    Start Switch Set 3
		  ss3[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 4:    //    Start Switch Set 4
		  ss4[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 5:    //    Start Switch Set 5
		  ss5[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 6:    //    Start Switch Set 6
		  ss6[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 7:    //    Start Switch Set 7
		  ss7[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 8:    //    Start Switch Set 8
		  ss8[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 9:    //    Start Switch Set 9
		  ss9[0] = 1;  run_switch_set[fpin]=true;
		break;
		case 10:    //    Start Switch Set 10
		  ss10[0] = 1;  run_switch_set[fpin]=true;
		break;
	  }
	}
	break;
	case 8:    // Start Switching set if not already started
	{
	  switch ( fpin )  {  // Start another Switching set based on the fpin argument
        case 0:    //  
		default:
		break;
        case 1:    //   Start Switch Set 1
		  if( run_switch_set[fpin]==false)  {ss1[0] = 1; run_switch_set[fpin]=true;}
		break;
        case 2:    //    Start Switch Set 2
		  if( run_switch_set[fpin]==false)  {ss2[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 3:    //    Start Switch Set 3
		  if( run_switch_set[fpin]==false)  {ss3[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 4:    //    Start Switch Set 4
		  if( run_switch_set[fpin]==false)  {ss4[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 5:    //    Start Switch Set 5
		  if( run_switch_set[fpin]==false)  {ss5[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 6:    //    Start Switch Set 6
		  if( run_switch_set[fpin]==false)  {ss6[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 7:    //    Start Switch Set 7
		  if( run_switch_set[fpin]==false)  {ss7[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 8:    //    Start Switch Set 8
		  if( run_switch_set[fpin]==false)  {ss8[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 9:    //    Start Switch Set 9
		  if( run_switch_set[fpin]==false)  {ss9[0] = 1; run_switch_set[fpin]=true;}
		break;
		case 10:    //    Start Switch Set 10
		  if( run_switch_set[fpin]==false)  {ss10[0] = 1; run_switch_set[fpin]=true;}
		break;
	  }
	}
	break;
	case 9:    // TBD
	
	break;
	case 10:   // TBD
	
	break;
	case 11:   // TBD
	
	break;
	case 12:   // TBD
	
	break;
	case 13:   // TBD
	
	break;
	case 14:   // TBD
	
	break;
	case 15:   // TBD
	
	break;
	case 16:   // TBD
	
	break;
	case 17:   // TBD
	
	break;
	case 18:   // TBD
	
	break;
	case 19:   // TBD
	
	break;
	case 20:   // TBD
	
	break;
	}
}     //   end    exec_switch_function()

boolean Weighted_ON()  {
	if (random (0, 100 ) > 40) return true;  //This will reyrn ON/true 60% of the time
	return false;
}    //   end    Weighted_ON()

void playTrackOnChannel ( byte dtrack)  {
#ifdef SOUND_PLAYER15
      if (dtrack == 0) return;
      if (dtrack!=127) {Player1.play(dtrack); delay(audiocmddelay); }
	    else {Player1.play(random(First_Track,Last_Track+1)); delay(audiocmddelay); }
#endif
}     //    end   playTrackOnChannel()

void setVolumeOnChannel ( byte dvolume)  {
#ifdef SOUND_PLAYER15
      if(dvolume>30) return;    // Don't change the volume if out of range
      Player1.volume (dvolume);
      delay(audiocmddelay);
#endif
}     //    end   setVolumeOnChannel()

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
}     // end  notifyDccFunc
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
    case 1:   //   run switch set [ function ]
    case 2:   //   run switch set [ function ]
    case 3:   //   run switch set [ function ]
    case 4:   //   run switch set [ function ]
    case 5:   //   run switch set [ function ]
    case 6:   //   run switch set [ function ]
    case 7:   //   run switch set [ function ]
    case 8:   //   run switch set [ function ]
    case 9:   //   run switch set [ function ]
    case 10:  //   run switch set [ function ]
      function_value[function] = byte(FuncState);
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
