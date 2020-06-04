// Production Stepper Drive DCC Decoder    Dec_Stepper_6Ftn.ino
// Version 6.01  Geoff Bunza 2014,2015,2016,2017,2018
// Now works with both short and long DCC Addesses

// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks
/*
 * Stepper Motor Drive (4 Pins Bi dirrectional) uses the 2 Motor controls MOT1 and MOT2 
 * F0 LED Pin 5
 * This is a “mobile/function” decoder that controls a single four wire stepper motor 
 * (5/12 Volt) via throttle speed setting and a multiplier which can be set in CV121.
 * Stepper speed is pre-set in the sketch but can be changed. The library also supports 
 * setting acceleration/deceleration for the stepper. The other functions are configurable
 * but are preset for LED on/off control. No servo motor control is available. 
 * Steppers whose coils need less than 500 ma can be accommodated. Each coil of the 
 * stepper attaches to MOT1 and MOT2. You may have to reverse the connections of one 
 * or the other until you get the connections right. The number of steps moved is set
 * by the speed setting multiplied by the contents of CV 121. 
 * Every Off to On activation of F2 will move the stepper the specified number of steps, 
 * in the direction set by the DCC speed direction.
*/
// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
#include <AccelStepper.h>
AccelStepper stepper(AccelStepper::FULL4WIRE, 3, 4, 9, 10);

int servo_slow_counter = 0; //servo loop counter to slowdown servo transit
long  Motor1Speed       = 0;
uint8_t Motor1ForwardDir  = 1;
uint8_t Motor1MaxSpeed    = 127;
long  Motor2Speed       = 0;
uint8_t Motor2ForwardDir  = 1;
uint8_t Motor2MaxSpeed    = 127;
int kickstarton = 1400;  //kick start cycle on time
int kickstarttime = 5;   //kick start duration on time
int fwdon = 0;
int fwdtime = 1;
int bwdon  = 0;
int bwdtime = 1;
int bwdshift = 0;
int cyclewidth = 2047;
int m2h = 3;    //R H Bridge     //Motor1
int m2l = 4;    //B H Bridge     //Motor1
int m0h = 9;    //R H Bridge     //Motor2
int m0l = 10;   //B H Bridge     //Motor2

int speedup = 112;   //Right track time differntial
int deltime = 1500;
int tim_delay = 100;
int numfpins = 17;
int num_active_fpins = 13;
byte fpins [] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
const int FunctionPin0 = 5;
const int FunctionPin1 = 6;
const int FunctionPin2 = 7;
const int FunctionPin3 = 8;
const int FunctionPin4 = 11;

const int FunctionPin5 = 12;
const int FunctionPin6 = 13;
const int FunctionPin7 = 14;     //A0
const int FunctionPin8 = 15;     //A1

const int FunctionPin9 = 16;     //A2
const int FunctionPin10 = 17;    //A3
const int FunctionPin11 = 18;    //A4
const int FunctionPin12 = 19;    //A5

byte Function2_value = 0;
int Function13_value = 0;
int Function14_value = 0;

NmraDcc  Dcc ;
DCC_MSG  Packet ;
uint8_t CV_DECODER_MASTER_RESET = 120;
uint8_t Motor_Multiplier = 121;
int t;  // temp

struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};
QUEUE *ftn_queue = new QUEUE[16];
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

  {CV_DECODER_MASTER_RESET, 0},     // CV 120
  {Motor_Multiplier, 10},           // CV 121
  {30, 0}, //F0 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {31, 1},    //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {32, 0},   //F0  Start Position F0=0
  {33, 8},  //F0  End Position   F0=1
  {34, 1},   //F0  Current Position
  {35, 0},  //F1 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {36, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {37, 0},   //  Start Position Fx=0
  {38, 8},  //  End Position   Fx=1
  {39, 1},  //  Current Position
  {40, 0},  //F2 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {41, 10},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {42, 28},   //  Start Position Fx=0
  {43, 140},  //  End Position   Fx=1
  {44, 0},    //  Current Position
  {45, 0}, //F3 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {46, 10},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {47, 28},   //  Start Position Fx=0
  {48, 140},  //  End Position   Fx=1
  {49, 0},    //  Current Position
  {50, 0}, //F4 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {51, 10},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {52, 28},    //  Start Position Fx=0
  {53, 140},    //  End Position   Fx=1
  {54, 0},    //  Current Position
  {55, 0}, //F5 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {56, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {57, 28},    //  Start Position Fx=0
  {58, 140},    //  End Position   Fx=1
  {59, 28},    //  Current Position
  {60, 0}, //F6 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {61, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {62, 1},    //  Start Position Fx=0
  {63, 255},    //  End Position   Fx=1
  {64, 28},    //  Current Position
  {65, 0}, //F7 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {66, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {67, 28},   //  Start Position Fx=0
  {68,140},  //  End Position   Fx=1
  {69, 28},    //  Current Position
  {70, 0}, //F8 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {71, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {72, 28},   //  Start Position Fx=0
  {73, 140},  //  End Position   Fx=1
  {74, 28},    //  Current Position
  {75, 0}, //F9 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {76, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {77, 28},   //  Start Position Fx=0
  {78, 140},  //  End Position   Fx=1
  {79, 28},    //  Current Position
  {80, 0}, //F10 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {81, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {82, 1},   //  Start Position Fx=0
  {83, 5},  //  End Position   Fx=1
  {84, 1},    //  Current Position
  {85, 0}, //F11 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {86, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 1},   //  Start Position Fx=0
  {88, 5},  //  End Position   Fx=1
  {89, 1},    //  Current Position
  {90, 0}, //F12 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {91, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {92, 1},   //  Start Position Fx=0
  {93, 10},  //  End Position   Fx=1
  {94, 1},    //  Current Position
  {95, 0}, //F13 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {96, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {97, 1},   //  Start Position Fx=0
  {98, 6},  //  End Position   Fx=1
  {99, 1},    //  Current Position
  {100, 0}, //F14 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {101, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {102, 1},   //  Start Position Fx=0
  {103, 6},  //  End Position   Fx=1
  {104, 1},    //  Current Position
  {105, 0}, //F15 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {106, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {107, 1},   //  Start Position Fx=0
  {108, 10},  //  End Position   Fx=1
  {109, 1},    //  Current Position
  {110, 0}, //F16 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {111, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {112, 1},   //  Start Position Fx=0
  {113, 10},  //  End Position   Fx=1
  {114, 1},    //  Current Position
//FUTURE USE
  {115, 0}, //F17 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {116, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {117, 28},   //  Start Position Fx=0
  {118, 50},  //  End Position   Fx=1
  {119, 28},    //  Current Position
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
  int i;
  uint8_t cv_value;
  // initialize the digital pins as outputs
    for (int i=0; i < numfpins; i++) {
      pinMode(fpins[i], OUTPUT);
      digitalWrite(fpins[i], 0);
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
         digitalWrite(fpins[14], 1);
         delay (1000);
         digitalWrite(fpins[14], 0);
     }
  for ( i=0; i < num_active_fpins; i++) {
    cv_value = Dcc.getCV( 30+(i*5)) ;   
#ifdef DEBUG
    Serial.print(" cv_value: ");
    Serial.println(cv_value, DEC) ;
#endif
    stepper.setMaxSpeed(100.0);
    stepper.setAcceleration(50.0);
    stepper.moveTo(1);
    switch ( cv_value ) {
      case 0:   // LED on/off
        ftn_queue[i].inuse = 0;
        break;
      case 1:   // LED Blink
         {
           ftn_queue[i].inuse = 0;
           ftn_queue[i].current_position = 0;
           ftn_queue[i].start_value = 0;
           ftn_queue[i].increment = int (char (Dcc.getCV( 31+(i*5))));
           digitalWrite(fpins[i], 0);
           ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5))) ;
         }
        break;
      case 2:   //servo  NOT AVAILABLE WITH THIS DECODER - STEPPER ONLY
        break;
      case 3:   // DOUBLE ALTERNATING LED Blink
         {
           ftn_queue[i].inuse = 0;
	         ftn_queue[i].current_position = 0;
           ftn_queue[i].start_value = 0;
           ftn_queue[i].increment = Dcc.getCV( 31+(i*5));
           digitalWrite(fpins[i], 0);
           digitalWrite(fpins[i+1], 0);
           ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5)));
         }
        break;
      case 4:   // Simple Pulsed Output based on saved Rate =10*Rate in Milliseconds
		   {
		   ftn_queue[i].inuse = 0;
		   ftn_queue[i].current_position = 0;
           ftn_queue[i].increment = 10 * int (char (Dcc.getCV( 31+(i*5))));
           digitalWrite(fpins[i], 0);
		     }
         break;
	     case 5:   // Fade On
         {
           ftn_queue[i].inuse = 0;
           ftn_queue[i].start_value = 0;
           ftn_queue[i].increment = int (char (Dcc.getCV( 31+(i*5))));
           digitalWrite(fpins[i], 0);
           ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5))) *10.;
         }
         break;         
       case 6:   // NEXT FEATURE to pin
         break;         
       default:
         break;
    }
  }
}
void loop()   //**********************************************************************
{
  //MUST call the NmraDcc.process() method frequently 
  // from the Arduino loop() function for correct library operation
  Dcc.process();
  delay(2);
  stepper.run();
  //*************************Normal Function Processing follows
  for (int i=0; i < num_active_fpins; i++) {
    if (ftn_queue[i].inuse==1)  {
    switch (Dcc.getCV( 30+(i*5))) {
      case 0:
        break;
      case 1:
	      ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          ftn_queue[i].start_value = ~ftn_queue[i].start_value;
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
          ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5)));
        }
        break;
      case 2:
        break;
      case 3:
	      ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          ftn_queue[i].start_value = ~ftn_queue[i].start_value;
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          digitalWrite(fpins[i]+1, ~ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
          ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5)));
        }
        i++;
        break;
       case 4:   // Simple Pulsed Output based on saved Rate =10*Rate in Milliseconds
		 {
		   ftn_queue[i].inuse = 0;
		   ftn_queue[i].current_position = 0;
           ftn_queue[i].increment = 10 * int (char (Dcc.getCV( 31+(i*5))));
           digitalWrite(fpins[i], 0);
		 }
         break;
	   case 5:   // Fade On

         break;         
       case 6:   // NEXT FEATURE to pin
         break;         
       default:
         break;  
      }
    }
  }
}
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION ForwardDir, DCC_SPEED_STEPS SpeedSteps )  {
   //if (Function13_value==1)  {
     Motor1Speed = Speed * Dcc.getCV( Motor_Multiplier);
     //Motor1ForwardDir  = ForwardDir & 1;
     if (ForwardDir == DCC_DIR_REV)  Motor1Speed = -Motor1Speed;;  
   //}
}
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
  if (FuncGrp==FN_0_4 && ((FuncState & FN_BIT_02)>>1) == 1)  {
    if (Function2_value == 0) {
      Function2_value=1;
      stepper.move(Motor1Speed);
      return;
    }
  } else if (FuncGrp==FN_0_4 && ((FuncState & FN_BIT_02)>>1) == 0)  {
    Function2_value = 0;
    return;
  }
  switch(FuncGrp)
  {
  case FN_0_4:    //Function Group 1 F0 F4 F3 F2 F1
	  exec_function( 0, FunctionPin0, (FuncState & FN_BIT_00)>>4 );
	  exec_function( 1, FunctionPin1, (FuncState & FN_BIT_01));
	  //exec_function( 2, FunctionPin2, (FuncState & FN_BIT_02)>>1);
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
//    Function13_value = (FuncState & FN_BIT_13);
//    Function14_value = (FuncState & FN_BIT_14)>>1;
//	  exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
//	  exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
      break;
	
  case FN_21_28:
      break;	
  }
}  
void exec_function (int function, int pin, int FuncState)  {
#ifdef DEBUG
    Serial.print(" function: ");
    Serial.println(function, DEC) ;
    Serial.print(" pin: ");
    Serial.println(pin, DEC) ;
    Serial.print(" FuncState: ");
    Serial.println(FuncState, DEC) ;
    Serial.print(" Dcc.getCV( 30+(function*5)): ");
    Serial.println(Dcc.getCV( 30+(function*5)), DEC) ;
#endif  
  if (function!=2) 
  switch ( Dcc.getCV( 30+(function*5)) )  {  // Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
    case 0:    // On - Off LED
      digitalWrite (pin, FuncState);
#ifdef DEBUG
    Serial.print(" Dcc.getCV( 30+(function*5)): ");
    Serial.println(Dcc.getCV( 30+(function*5)), DEC) ;
    Serial.print(" pin: ");
    Serial.println(pin, DEC) ;
    Serial.print(" FuncState: ");
    Serial.println(FuncState, DEC) ;
#endif  
      ftn_queue[function].inuse = 0;
      break;
    case 1:    // Blinking LED
      if ((ftn_queue[function].inuse==0) && (FuncState==1))  {
        ftn_queue[function].inuse = 1;
        ftn_queue[function].start_value = 0;
        digitalWrite(pin, 0);
        ftn_queue[function].stop_value = int(Dcc.getCV( 33+(function*5)));
      } else {
          if ((ftn_queue[function].inuse==1) && (FuncState==0)) {
            ftn_queue[function].inuse = 0;
            digitalWrite(pin, 0);
          }
        }
      break;
    case 2:    // Servo
      break;
    case 3:    // Blinking LED PAIR
      if ((ftn_queue[function].inuse==0) && (FuncState==1))  {
        ftn_queue[function].inuse = 1;
        ftn_queue[function].start_value = 0;
        digitalWrite(fpins[function], 0);
        digitalWrite(fpins[function+1], 1);
        ftn_queue[function].stop_value = int(Dcc.getCV( 33+(function*5)));
      } else {
          if (FuncState==0) {
            ftn_queue[function].inuse = 0;
            digitalWrite(fpins[function], 0);
            digitalWrite(fpins[function+1], 0);
          }
        }
      break;
    case 4:    // Pulse Output based on Rate*10 Milliseconds
      if ((ftn_queue[function].inuse==0) && (FuncState==1)) {  //First Turn On Detected
        digitalWrite(fpins[function], 1);
	    	delay (10*ftn_queue[function].increment);
        digitalWrite(fpins[function], 0);
		ftn_queue[function].inuse = 1;                    //inuse set to 1 says we already pulsed
      } else 
          if (FuncState==0)  ftn_queue[function].inuse = 0;
      break;	  
    case 5:    // Fade On
#define fadedelay 24
      if ((ftn_queue[function].inuse==0) && (FuncState==1))  {
        ftn_queue[function].inuse = 1;
        for (t=0; t<ftn_queue[function].stop_value; t+=ftn_queue[function].increment) {
          digitalWrite( fpins[function], 1);
          delay(fadedelay*(t/(1.*ftn_queue[function].stop_value)));
          digitalWrite( fpins[function], 0);
          delay(fadedelay-(fadedelay*(t/(1.*ftn_queue[function].stop_value))));
        }
        digitalWrite( fpins[function],  1 );
      } else {
          if ((ftn_queue[function].inuse==1) && (FuncState==0)) {
            ftn_queue[function].inuse = 0;
            digitalWrite(fpins[function], 0);
          }
        }
      break;
    case 6:    // Future Function
      ftn_queue[function].inuse = 0;
      break;
    default:
      ftn_queue[function].inuse = 0;
      break;
  }
}
