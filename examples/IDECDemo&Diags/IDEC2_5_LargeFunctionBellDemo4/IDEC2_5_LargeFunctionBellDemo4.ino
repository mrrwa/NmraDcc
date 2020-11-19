// Interactive Decoder Random Switches   IDEC2_5_LargeFunctionSetsDev.ino
// Version 1.09 Geoff Bunza 2020
// Works with both short and long DCC Addesses
// This decoder will control Switch Sequences, servos, sounds and LEDs
// F0=Master Function OFF = Function ON DISABLES the decoder
/* F0 is configured as Master Decoder Disable Override  ON==Disable Decoder
 * Input Pin for Decoder Disable  Pin 16/A2 Active LOW
// F1-F2 Eight Switch Sets 1-2 controlled by Input pins 3,4  respectively
 *  All Switch Sets are defined by groups of 80 CVs
  - Either a DCC Function 1-2 on OR an Input Pin (3,4,) Switched Low enables a decoder function (ON)
  - BOTH the respective DCC Decoder Function 1-2 must be Off AND its respective Input Pin  (3,4) 
    MUST be High for a decoder function to be considered disabled
  - A decoder function LEFT ENABLED will repreat the respecpective action as long as it is enabled
 * Switch Set CV's are 25 groups of 3 CVs each:  
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
 * Switch sets include CVs:  50-129 and 130=209
 * MAX one of 3 Configurations per pin function:
 *  Config 0=DISABLE On/Off,1-2=Switch Control 1-2
 
PRO MINI PIN ASSIGNMENT:
2 - DCC Input
3 - Input Pin Switch   1
4 - Input Pin Switch   2
5 - Switch 9
6 - Switch 10
7 - Switch 11
8 - Switch 12
9 - Switch 13
10 - Switch 14
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
#define SOUND_PLAYER15

// ******** REMOVE THE "//" IN THE FOLOWING LINE TO enable SERVO USE
#define USE_SERVO14

// ******** REMOVE THE "//" IN THE FOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR  *****NOTE Turning DEBUG ON changes ALL Timing!!
//#define DEBUG


#include <NmraDcc.h>
#include <SoftwareServo.h>

#ifdef USE_SERVO14
SoftwareServo sservo;
#endif
byte default_servo_pin = 14;
byte servo_start_position = 90;
const byte servo_master_slowdown_CV = 48;   //servo loop counter limit CV
int servo_slow_counter = 1; //servo loop counter to slowdown servo transit
byte servo_unit_faster = 0;

#ifdef SOUND_PLAYER15
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
SoftwareSerial DFSerial1(22,15); // PRO MINI RX, PRO MINI TX serial to DFPlayer
DFRobotDFPlayerMini Player1;
#endif
#define  default_volume  20    //  sets default volume 0-30,  0 == OFF,  >30 == Skip Track
#define First_Track 1   // Play Random Tracks First_Track#=Start_Track  >=1
#define Last_Track 12   // Play Random Tracks Last_Track= Last Playable Track in Range  <= Last Numbered Track
const byte starting_volume_CV  = 49;   // If no volume is set use this CV at the start
const int audiocmddelay = 40;

#define This_Decoder_Address 24
uint8_t CV_DECODER_MASTER_RESET = 252;

//Uncomment ONLY ONE of the following:
//#define  MasterTimeConstant  1L       // milliseconds Timing
#define  MasterTimeConstant  10L      // 10's of milliseconds Timing
//#define  MasterTimeConstant  100L     // Tenths of a second Timing
//#define  MasterTimeConstant  1000L    // Seconds Timing
//#define  MasterTimeConstant  10000L   // 10's of Seconds Timing
//#define  MasterTimeConstant  60000L   // Minutes Timing
//#define  MasterTimeConstant  3600000L // Hours Timing

int del_tim = 4000; 
uint16_t ttemp, i;
byte otemp;

byte ss1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long ss1delay=0;
byte ss2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long ss2delay=0;
bool run_switch_set [ ] = {false,false,false};
byte switchset_channel[ ]={0,0,0};

struct SERVO_TRACKING
{
  byte current_position;
  byte target_position;
  char delta;
};
SERVO_TRACKING servo_pos;

const int MasterDecoderDisablePin = 16; // D16/A0  Master Decoder Disable Input Pin Active LOW

const int numINpins = 3;              // Number of INput pins to initialize
byte inputpins [] = {3,4,16};  //These are all the Input Pins
const int numfpins = 14;              // Number of Output pins to initialize
const int num_active_functions = 3;   // Number of Functions stating with F0
byte fpins [] = {5,6,7,8,9,10,11,12,13,14,15,17,18,19};  //These are all the Output Pins (first 15 is placeholder)
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
#define switch9 5
#define switch10 6
#define switch11 7
#define switch12 8
#define switch13 9
#define switch14 10
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

  {48, 120}, // servo_master_slowdown_CV Higher number slows servo movment
  {49, 22},  // Starting sound volume (starting_volume_CV)
  
#define soundM      4
#define servoM      5
#define delayM      55
#define servostartM 140
#define servoendM   42
  {50, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 1-1
  {51, servoM}, // servo Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {52, servostartM}, // servo position
  {53, delayM}, // Wait2 0-254 0.1 Seconds
  {54, soundM},  // sound Mode
  {55, 1},  // sound track
  {56, delayM}, // Wait3 0-254 0.1 Seconds
  {57, servoM}, // servo Mode
  {58, servoendM}, // servo position
  {59, delayM},  // Wait4 0-254 0.1 Seconds
  {60, soundM}, // sound Mode
  {61, 2}, // sound track
  {62, delayM}, // Wait5 0-254 0.1 Seconds
  {63, servoM}, // servo Mode
  {64, servostartM},  // servo position
  {65, 0}, // Not Used
  
  {66, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 1-2
  {67, soundM}, // sound Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {68, 3}, // sound track
  {69, delayM}, // Wait2 0-254 0.1 Seconds
  {70, servoM},  // servo Mode
  {71, servoendM}, // servo position
  {72, delayM}, // Wait3 0-254 0.1 Seconds
  {73, soundM}, // sound Mode
  {74, 4}, // sound track
  {75, delayM},  // Wait4 0-254 0.1 Seconds
  {76, servoM}, // servo  Mode
  {77, servostartM}, // servo position
  {78, delayM}, // Wait5 0-254 0.1 Seconds
  {79, soundM}, // sound Mode
  {80, 5},  // sound track
  {81, 0}, // Not Used
  
  {82, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 1-3
  {83, servoM}, // servo Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {84, servoendM}, // servo
  {85, delayM}, // Wait2 0-254 0.1 Seconds
  {86, soundM},  // sound Mode
  {87, 6}, // sound
  {88, delayM}, // Wait3 0-254 0.1 Seconds
  {89, servoM}, // servo Mode
  {90, servostartM}, // servo
  {91, delayM},  // Wait4 0-254 0.1 Seconds
  {92, soundM}, // sound Mode
  {93, 7}, // sound
  {94, delayM}, // Wait5 0-254 0.1 Seconds
  {95, servoM}, // servo Mode
  {96, servoendM},  // servo
  {97, 0}, // Not Used

  {98, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 1-4
  {99,  soundM}, // sound Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {100, 8}, // sound
  {101, delayM}, // Wait2 0-254 0.1 Seconds
  {102, servoM},  // servo Mode
  {103, servostartM}, // servo
  {104, delayM}, // Wait3 0-254 0.1 Seconds
  {105, soundM}, // sound Mode
  {106, 9}, // sound
  {107, delayM},  // Wait4 0-254 0.1 Seconds
  {108, servoM}, // servo Mode
  {109, servoendM}, // Switch Pin4
  {110, delayM}, // Wait5 0-254 0.1 Seconds
  {111, soundM}, // sound Mode
  {112, 10},  // sound
  {113, 0}, // Not Used
  
  {114, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 1-5
  {115, servoM}, // servo Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {116, servostartM}, // servo
  {117, delayM}, // Wait2 0-254 0.1 Seconds
  {118, soundM},  // sound Mode
  {119, 11}, // sound
  {120, delayM}, // Wait3 0-254 0.1 Seconds
  {121, servoM}, // servo Mode
  {122, servoendM}, // servo
  {123, delayM},  // Wait4 0-254 0.1 Seconds
  {124, soundM}, // sound Mode
  {125, 12}, // sound
  {126, delayM}, // Wait5 0-254 0.1 Seconds
  {127, servoM}, // servo Mode
  {128, 90},  // servo
  {129, 0}, // Not Used
  
  {130, 0}, // Wait1 0-254 0.1 Seconds  // switch SET 2-1
  {131, servoM}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {132, servostartM}, // Switch Pin1
  {133, delayM}, // Wait2 0-254 0.1 Seconds
  {134, soundM},  // Switch Mode
  {135, 13}, // Switch Pin2
  {136, delayM}, // Wait3 0-254 0.1 Seconds
  {137, servoM}, // Switch Mode
  {138, servoendM}, // Switch Pin3
  {139, delayM},  // Wait4 0-254 0.1 Seconds
  {140, soundM}, // Switch Mode
  {141, 14}, // Switch Pin4
  {142, delayM}, // Wait5 0-254 0.1 Seconds
  {143, servoM}, // Switch Mode
  {144, servostartM},  // Switch Pin5
  {145, 0}, // Not Used
  
  {146, delayM}, // Wait1 0-254 0.1 Seconds // switch SET 2-2
  {147, soundM}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {148, 15}, // Switch Pin1
  {149, delayM}, // Wait2 0-254 0.1 Seconds
  {150, servoM},  // Switch Mode
  {151, servoendM}, // Switch Pin2
  {152, delayM}, // Wait3 0-254 0.1 Seconds
  {153, soundM}, // Switch Mode
  {154, 16}, // Switch Pin3
  {155, delayM},  // Wait4 0-254 0.1 Seconds
  {156, servoM}, // Switch Mode
  {157, servostartM}, // Switch Pin4
  {158, delayM}, // Wait5 0-254 0.1 Seconds
  {159, soundM}, // Switch Mode
  {160, 17},  // Switch Pin5
  {161, 0}, // Not Used
  
  {162, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 2-3
  {163, servoM}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {164, servoendM}, // Switch Pin1
  {165, delayM}, // Wait2 0-254 0.1 Seconds
  {166, soundM},  // Switch Mode
  {167, 0}, // Switch Pin2
  {168, delayM}, // Wait3 0-254 0.1 Seconds
  {169, servoM}, // Switch Mode
  {170, servostartM}, // Switch Pin3
  {171, delayM},  // Wait4 0-254 0.1 Seconds
  {172, soundM}, // Switch Mode
  {173, 0}, // Switch Pin4
  {174, delayM}, // Wait5 0-254 0.1 Seconds
  {175, servoM}, // Switch Mode
  {176, servoendM},  // Switch Pin5
  {177, 0}, // Not Used
  
  {178, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 2-4
  {179, soundM}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {180, 0}, // Switch Pin1
  {181, delayM}, // Wait2 0-254 0.1 Seconds
  {182, servoM},  // Switch Mode
  {183, servostartM}, // Switch Pin2
  {184, delayM}, // Wait3 0-254 0.1 Seconds
  {185, soundM}, // Switch Mode
  {186, 0}, // Switch Pin3
  {187, delayM},  // Wait4 0-254 0.1 Seconds
  {188, servoM}, // Switch Mode
  {189, servoendM}, // Switch Pin4
  {190, delayM}, // Wait5 0-254 0.1 Seconds
  {191, soundM}, // Switch Mode
  {192, 0},  // Switch Pin5
  {193, 0}, // Not Used
  
  {194, delayM}, // Wait1 0-254 0.1 Seconds  // switch SET 2-5
  {195, servoM}, // Switch Mode              // 0=NOP,1=0/1,2=RND,3=WRND,4=SND,5=SRVO,6=Dual Pin,7=Next SS,8=Next SS Conditional
  {196, servostartM}, // Switch Pin1
  {197, delayM}, // Wait2 0-254 0.1 Seconds
  {198, soundM},  // Switch Mode
  {199, 0}, // Switch Pin2
  {200, delayM}, // Wait3 0-254 0.1 Seconds
  {201, servoM}, // Switch Mode
  {202, servoendM}, // Switch Pin3
  {203, delayM},  // Wait4 0-254 0.1 Seconds
  {204, soundM}, // Switch Mode
  {205, 0}, // Switch Pin4
  {206, delayM}, // Wait5 0-254 0.1 Seconds
  {207, servoM}, // Switch Mode
  {208, 90},  // Switch Pin5
  {209, 0}, // Not Used

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
  pinMode (MasterDecoderDisablePin,INPUT_PULLUP); //Master Decoder Disable Input Active LOW
  // initialize the digital pins as outputs
  for (int i=0; i < numfpins; i++) {
    pinMode(fpins[i], OUTPUT);
    digitalWrite(fpins[i], 0);    // All OUPUT pins initialized LOW
   }
  // initialize the digital pins as inputs
  for (int i=0; i < numINpins; i++) {
    pinMode(inputpins[i], INPUT_PULLUP);
  }
#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef SOUND_PLAYER15
  DFSerial1.begin (9600);
  delay(40);
  Player1.begin (DFSerial1);
  delay(80);
  setVolumeOnChannel (Dcc.getCV(starting_volume_CV));
#endif

#ifdef USE_SERVO14
  sservo.attach(default_servo_pin);  // Start Servo on default_servo_pin  //Position Servo
  delay(50);
  sservo.write(servo_start_position);  // Set starting position
  servo_pos.current_position = servo_start_position;
  servo_pos.delta = 0;
  servo_pos.target_position = servo_start_position;
  delay(500);
  SoftwareServo::refresh();
#endif

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
    Serial.println("Large Switch Set 1");
    for (i=50; i<66; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=50; i<66; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=66; i<82; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=66; i<82; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=82; i<98; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=82; i<98; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=98; i<114; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=98; i<114; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=114; i<130; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=114; i<130; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    Serial.println("Large Switch Set 2");
    for (i=130; i<146; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=130; i<146; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=146; i<162; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=146; i<162; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=162; i<178; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=162; i<178; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=178; i<194; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=178; i<194; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=194; i<210; i++) { Serial.print(i,DEC); Serial.print("\t"); }
    Serial.println("");
    for (i=194; i<210; i++) { Serial.print(Dcc.getCV(i),DEC); Serial.print("\t"); }
    Serial.println("");
#endif
}
void loop()   //***********************************************************************************
{
  //MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
#ifdef USE_SERVO14
  SoftwareServo::refresh();
  if (servo_slow_counter > Dcc.getCV(servo_master_slowdown_CV) )  update_servos();
    else servo_slow_counter++;
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
          }
      break;
      case 2:   //   
         if (((digitalRead(4)==LOW)||(function_value[cv_value]==1)) && !run_switch_set[cv_value]) {
           ss2[0]=1; run_switch_set[cv_value]=true; 
           }
      break;
      default:  //  Extra
      break;
		}
      }
    }
   //    ==========================   Large switch Set 1 Start Run
  if (ss1[0]==1) {
     ss1delay=millis()+(long(Dcc.getCV(50)*MasterTimeConstant)); // Wait1
     ss1[0]=0;  ss1[1]=1;  
    }
  if ((ss1[1]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(52));
    exec_switch_function(Dcc.getCV(51),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(53)*MasterTimeConstant)); // Wait2
    ss1[1]=0;  ss1[2]=1;
    }
  if ((ss1[2]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(55));
      exec_switch_function (Dcc.getCV(54),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(56)*MasterTimeConstant)); // Wait3
      ss1[2]=0;  ss1[3]=1;
    }
  if ((ss1[3]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(58));
      exec_switch_function(Dcc.getCV(57),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(59)*MasterTimeConstant)); // Wait4
      ss1[3]=0;  ss1[4]=1;
    }
  if ((ss1[4]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(61));
      exec_switch_function(Dcc.getCV(60),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(62)*MasterTimeConstant)); // Wait5
      ss1[4]=0;  ss1[5]=1;
    }
  if ((ss1[5]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(64));
      exec_switch_function(Dcc.getCV(63),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss1[5]=0;  ss1[6]=1;
    }
//    ==========================   Large switch Set 1 continues
  if (ss1[6]==1) {
     ss1delay=millis()+(long(Dcc.getCV(66)*MasterTimeConstant)); // Wait1
     ss1[6]=0;  ss1[7]=1;
    }
  if ((ss1[7]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(68));
    exec_switch_function(Dcc.getCV(67),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(69)*MasterTimeConstant)); // Wait2
    ss1[7]=0;  ss1[8]=1;
    }
  if ((ss1[8]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(71));
      exec_switch_function(Dcc.getCV(70),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(72)*MasterTimeConstant)); // Wait3
      ss1[8]=0;  ss1[9]=1;
    }
  if ((ss1[9]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(74));
      exec_switch_function(Dcc.getCV(73),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(75)*MasterTimeConstant)); // Wait4
      ss1[9]=0;  ss1[10]=1;
    }
  if ((ss1[10]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(77));
      exec_switch_function(Dcc.getCV(76),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(78)*MasterTimeConstant)); // Wait5
      ss1[10]=0;  ss1[11]=1;
    }
  if ((ss1[11]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(80));
      exec_switch_function(Dcc.getCV(79),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss1[11]=0;  ss1[12]=1;
    }
//    ==========================   Large switch Set 1 continues
  if (ss1[12]==1) {
     ss1delay=millis()+(long(Dcc.getCV(82)*MasterTimeConstant)); // Wait1
     ss1[12]=0;  ss1[13]=1;
    }
  if ((ss1[13]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(84));
    exec_switch_function(Dcc.getCV(83),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(85)*MasterTimeConstant)); // Wait2
    ss1[13]=0;  ss1[14]=1;
    }
  if ((ss1[14]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(87));
      exec_switch_function(Dcc.getCV(86),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(88)*MasterTimeConstant)); // Wait3
      ss1[14]=0;  ss1[15]=1;
    }
  if ((ss1[15]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(90));
      exec_switch_function(Dcc.getCV(89),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(91)*MasterTimeConstant)); // Wait4
      ss1[15]=0;  ss1[16]=1;
    }
  if ((ss1[16]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(93));
      exec_switch_function(Dcc.getCV(92),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(94)*MasterTimeConstant)); // Wait5
      ss1[16]=0;  ss1[17]=1;
    }
  if ((ss1[17]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(96));
      exec_switch_function(Dcc.getCV(95),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss1[17]=0;   ss1[18]=1;
    }
//    ==========================  Large switch Set 1 continues
  if (ss1[18]==1) {
     ss1delay=millis()+(long(Dcc.getCV(98)*MasterTimeConstant)); // Wait1
     ss1[18]=0;  ss1[19]=1;
    }
  if ((ss1[19]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(100));
    exec_switch_function(Dcc.getCV(99),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(101)*MasterTimeConstant)); // Wait2
    ss1[19]=0;  ss1[20]=1;
    }
  if ((ss1[20]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(103));
      exec_switch_function(Dcc.getCV(102),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(104)*MasterTimeConstant)); // Wait3
      ss1[20]=0;  ss1[21]=1;
    }
  if ((ss1[21]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(106));
      exec_switch_function(Dcc.getCV(105),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(107)*MasterTimeConstant)); // Wait4
      ss1[21]=0;  ss1[22]=1;
    }
  if ((ss1[22]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(109));
      exec_switch_function(Dcc.getCV(108),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(110)*MasterTimeConstant)); // Wait5
      ss1[22]=0;  ss1[23]=1;
    }
  if ((ss1[23]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(112));
      exec_switch_function(Dcc.getCV(111),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss1[23]=0;   ss1[24]=1;
    }
//    ==========================   Large switch Set 1 continues
  if (ss1[24]==1) {
     ss1delay=millis()+(long(Dcc.getCV(114)*MasterTimeConstant)); // Wait1
     ss1[24]=0;  ss1[25]=1;
    }
  if ((ss1[25]==1)&&(ss1delay<=millis())) {
    ttemp=(Dcc.getCV(116));
    exec_switch_function(Dcc.getCV(115),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss1delay=millis()+(long(Dcc.getCV(117)*MasterTimeConstant)); // Wait2
    ss1[25]=0;  ss1[26]=1;
    }
  if ((ss1[26]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(119));
      exec_switch_function(Dcc.getCV(118),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss1delay=millis()+(long(Dcc.getCV(120)*MasterTimeConstant)); // Wait3
      ss1[26]=0;  ss1[27]=1;
    }
  if ((ss1[27]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(122));
      exec_switch_function(Dcc.getCV(121),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss1delay=millis()+(long(Dcc.getCV(123)*MasterTimeConstant)); // Wait4
      ss1[27]=0;  ss1[28]=1;
    }
  if ((ss1[28]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(125));
      exec_switch_function(Dcc.getCV(124),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss1delay=millis()+(long(Dcc.getCV(126)*MasterTimeConstant)); // Wait5
      ss1[28]=0;  ss1[29]=1;
    }
  if ((ss1[29]==1)&&(ss1delay<=millis())) {
      ttemp=(Dcc.getCV(128));
      exec_switch_function(Dcc.getCV(127),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss1[29]=0;    run_switch_set[1]=false;
    }
  
//    ==========================   Large switch Set 2 Start Run
  if (ss2[0]==1) {
     ss2delay=millis()+(long(Dcc.getCV(130)*MasterTimeConstant)); // Wait1
     ss2[0]=0;  ss2[1]=1;
    }
  if ((ss2[1]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(132));
    exec_switch_function(Dcc.getCV(131),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(133)*MasterTimeConstant)); // Wait2
    ss2[1]=0;  ss2[2]=1;
    }
  if ((ss2[2]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(135));
      exec_switch_function(Dcc.getCV(134),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(136)*MasterTimeConstant)); // Wait3
      ss2[2]=0;  ss2[3]=1;
    }
  if ((ss2[3]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(138));
      exec_switch_function(Dcc.getCV(137),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(139)*MasterTimeConstant)); // Wait4
      ss2[3]=0;  ss2[4]=1;
    }
  if ((ss2[4]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(141));
      exec_switch_function(Dcc.getCV(140),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(142)*MasterTimeConstant)); // Wait5
      ss2[4]=0;  ss2[5]=1;
    }
  if ((ss2[5]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(144));
      exec_switch_function(Dcc.getCV(143),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss2[5]=0;   ss2[6]=1;
    }
  
//    ==========================   switch Set 7 Start Run
  if (ss2[6]==1) {
     ss2delay=millis()+(long(Dcc.getCV(146)*MasterTimeConstant)); // Wait1
     ss2[6]=0;  ss2[7]=1;
    }
  if ((ss2[7]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(148));
    exec_switch_function(Dcc.getCV(147),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(149)*MasterTimeConstant)); // Wait2
    ss2[7]=0;  ss2[8]=1;
    }
  if ((ss2[8]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(151));
      exec_switch_function(Dcc.getCV(150),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(152)*MasterTimeConstant)); // Wait3
      ss2[8]=0;  ss2[9]=1;
    }
  if ((ss2[9]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(154));
      exec_switch_function(Dcc.getCV(153),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(155)*MasterTimeConstant)); // Wait4
      ss2[9]=0;  ss2[10]=1;
    }
  if ((ss2[10]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(157));
      exec_switch_function(Dcc.getCV(156),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(158)*MasterTimeConstant)); // Wait5
      ss2[10]=0;  ss2[11]=1;
    }
  if ((ss2[11]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(160));
      exec_switch_function(Dcc.getCV(159),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss2[11]=0;   ss2[12]=1;
    }
//    ==========================   switch Set 8 Start Run
  if (ss2[12]==1) {
     ss2delay=millis()+(long(Dcc.getCV(162)*MasterTimeConstant)); // Wait1
     ss2[12]=0;  ss2[13]=1;
    }
  if ((ss2[13]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(164));
    exec_switch_function(Dcc.getCV(163),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(165)*MasterTimeConstant)); // Wait2
    ss2[13]=0;  ss2[14]=1;
    }
  if ((ss2[14]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(167));
      exec_switch_function(Dcc.getCV(166),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(168)*MasterTimeConstant)); // Wait3
      ss2[14]=0;  ss2[15]=1;
    }
  if ((ss2[15]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(170));
      exec_switch_function(Dcc.getCV(169),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(171)*MasterTimeConstant)); // Wait4
      ss2[15]=0;  ss2[16]=1;
    }
  if ((ss2[16]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(173));
      exec_switch_function(Dcc.getCV(172),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(174)*MasterTimeConstant)); // Wait5
      ss2[16]=0;  ss2[17]=1;
    }
  if ((ss2[17]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(176));
      exec_switch_function(Dcc.getCV(175),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss2[17]=0;   ss2[18]=1;
    }
//    ==========================   switch Set 9 Start Run
  if (ss2[18]==1) {
     ss2delay=millis()+(long(Dcc.getCV(178)*MasterTimeConstant)); // Wait1
     ss2[18]=0;  ss2[19]=1;
    }
  if ((ss2[19]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(180));
    exec_switch_function(Dcc.getCV(179),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(181)*MasterTimeConstant)); // Wait2
    ss2[19]=0;  ss2[20]=1;
    }
  if ((ss2[20]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(183));
      exec_switch_function(Dcc.getCV(182),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(184)*MasterTimeConstant)); // Wait3
      ss2[20]=0;  ss2[21]=1;
    }
  if ((ss2[21]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(186));
      exec_switch_function(Dcc.getCV(185),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(187)*MasterTimeConstant)); // Wait4
      ss2[21]=0;  ss2[22]=1;
    }
  if ((ss2[22]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(189));
      exec_switch_function(Dcc.getCV(188),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(190)*MasterTimeConstant)); // Wait5
      ss2[22]=0;  ss2[23]=1;
    }
  if ((ss2[23]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(192));
      exec_switch_function(Dcc.getCV(191),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss2[23]=0;   ss2[24]=1;
    }
//    ==========================   switch Set 10 Start Run
  if (ss2[24]==1) {
     ss2delay=millis()+(long(Dcc.getCV(194)*MasterTimeConstant)); // Wait1
     ss2[24]=0;  ss2[25]=1;
    }
  if ((ss2[25]==1)&&(ss2delay<=millis())) {
    ttemp=(Dcc.getCV(196));
    exec_switch_function(Dcc.getCV(195),ttemp&0x7f,ttemp>>7);  //  execute switch function 1
    ss2delay=millis()+(long(Dcc.getCV(197)*MasterTimeConstant)); // Wait2
    ss2[25]=0;  ss2[26]=1;
    }
  if ((ss2[26]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(199));
      exec_switch_function(Dcc.getCV(198),ttemp&0x7f,ttemp>>7);  //  execute switch function 2
      ss2delay=millis()+(long(Dcc.getCV(200)*MasterTimeConstant)); // Wait3
      ss2[26]=0;  ss2[27]=1;
    }
  if ((ss2[27]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(202));
      exec_switch_function(Dcc.getCV(201),ttemp&0x7f,ttemp>>7);  //  execute switch function 3
      ss2delay=millis()+(long(Dcc.getCV(203)*MasterTimeConstant)); // Wait4
      ss2[27]=0;  ss2[28]=1;
    }
  if ((ss2[28]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(205));
      exec_switch_function(Dcc.getCV(204),ttemp&0x7f,ttemp>>7);  //  execute switch function  4
      ss2delay=millis()+(long(Dcc.getCV(206)*MasterTimeConstant)); // Wait5
      ss2[28]=0;  ss2[29]=1;
    }
  if ((ss2[29]==1)&&(ss2delay<=millis())) {
      ttemp=(Dcc.getCV(208));
      exec_switch_function(Dcc.getCV(207),ttemp&0x7f,ttemp>>7);  //  execute switch function  5
      ss2[29]=0;    run_switch_set[2]=false;
    }
}       // end  loop() 

void exec_switch_function (byte switch_function, byte fpin,byte fbit)  {
  byte arg;
  if (MasterDecoderDisable == 1) return;
  
  arg = fpin|(fbit<<7);              // 8 bits of argument passed
  
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
	  //setVolumeOnChannel (default_volume * fbit); //  set volume level
	  playTrackOnChannel(fpin);                   //  play sound track  fpin 1-127
#endif
	break;
	case 5:    //
#ifdef USE_SERVO14
    set_servo(arg);                // Position Servo
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
	  }
	}
	break;
	}
 SoftwareServo::refresh();
}     //   end    exec_switch_function()

boolean Weighted_ON()  {
	if (random (0, 100 ) > 40) return true;  //This will reyrn ON/true 60% of the time
	return false;
}    //   end    Weighted_ON()

#ifdef USE_SERVO14
void set_servo (byte position)  {
  if (servo_pos.current_position == position) {
    servo_pos.delta = 0;
    servo_pos.target_position = position;
    return;
  }
  if (servo_pos.current_position < position)  servo_pos.delta = 1+servo_unit_faster;
    else servo_pos.delta = -1-servo_unit_faster;
    servo_pos.target_position = position;
}      //   end  set_servo()

void update_servos()  {
  servo_slow_counter = 1;
    if (servo_pos.delta != 0 ) {
      servo_pos.current_position=servo_pos.current_position+servo_pos.delta;
      if ((servo_pos.delta > 0)&&(servo_pos.current_position > servo_pos.target_position)) 
      {
        servo_pos.current_position = servo_pos.target_position;
        servo_pos.delta = 0;
      }
      if ((servo_pos.delta < 0)&&(servo_pos.current_position < servo_pos.target_position))  
      {
        servo_pos.current_position = servo_pos.target_position;
        servo_pos.delta = 0;
      }
      if (servo_pos.current_position == servo_pos.target_position) servo_pos.delta = 0;
    }
  sservo.write ( servo_pos.current_position );
}     //    end  update_servos()
#endif

void playTrackOnChannel ( byte dtrack)  {
#ifdef SOUND_PLAYER15
      if (dtrack == 0) return;
      if (dtrack!=127) {Player1.play(dtrack); }  //delay(audiocmddelay); }
	    else {Player1.play(random(First_Track,Last_Track+1)); } //delay(audiocmddelay); }
#endif
}     //    end   playTrackOnChannel()

void setVolumeOnChannel ( byte dvolume)  {
#ifdef SOUND_PLAYER15
      if(dvolume>30) return;    // Don't change the volume if out of range
      Player1.volume(dvolume);
      delay(audiocmddelay);
#endif
}     //    end   setVolumeOnChannel()

void    notifyCVChange( uint16_t CV, uint8_t Value)  {
#ifdef DEBUG
   //Serial.print("CV= ");
   //Serial.println(CV, DEC) ;
   //Serial.print("Value= ");
   //Serial.println(Value, DEC) ;
#endif
  if ( CV==49 )  setVolumeOnChannel (Value); 
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
