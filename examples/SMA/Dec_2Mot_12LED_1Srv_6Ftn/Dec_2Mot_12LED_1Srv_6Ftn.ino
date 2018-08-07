// Production 2 Motor 13 Function DCC Decoder    Dec_2MotDrive_12LED_1Srv_6Ftn.ino
// Version 6.01a  Geoff Bunza 2014,2015,2016,2017,2018
// Now works with both short and long DCC Addesses
// Better motor control added
// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks
/*
 * Motor selection is via motor select Function 13 (Motor1) and Function 14 (Motor2)  
 * Motor speed for each can only be changed if the corresponding Function is on 
 * (F13 and/or F14). Motor speed is maintained if the corresponding Motor select function 
 * is off. Thus, each motor can be controlled independently and run at different speeds.
 *  F0-F12 control LEDs on Pro Mini Digital Pins 5,6,7,8,11,12,13,14,15,16,17,18,19
 *  Simple speed control is made via throttle speed setting for two motors. Motor selection 
 *  is via motor select Function 13 (Motor1) and Function 14 (Motor2).  Motor speed for each 
 *  can only be changed if the corresponding Function is on (F13 and/or F14). Motor speed is
 *  maintained if the corresponding motor select function is off. Thus, each motor can be 
 *  controlled independently and run at different speeds. The other functions are configurable 
 *  but are preset for LED on/off control.
*/
// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
#include <SoftwareServo.h> 

SoftwareServo servo[13];
#define servo_start_delay 50
#define servo_init_delay 7
#define servo_slowdown  12   //servo loop counter limit
int servo_slow_counter = 0; //servo loop counter to slowdown servo transit

int     Motor1Speed       = 0;
uint8_t Motor1ForwardDir  = 1;
int     Motor2Speed       = 0;
uint8_t Motor2ForwardDir  = 1;
int kickstarton = 1400;  //kick start cycle on time
int kickstarttime = 5;   //kick start duration on time
int fwdon = 0;
int fwdtime = 1;
int bwdon  = 0;
int bwdtime = 1;
int bwdshift = 0;
int cyclewidth = 2047;
int loopdelay =14;
int m2h = 3;    //R H Bridge     //Motor1
int m2l = 4;    //B H Bridge     //Motor1
int m0h = 9;    //R H Bridge     //Motor2
int m0l = 10;   //B H Bridge     //Motor2

int speedup = 112;   //Right track time differntial
int deltime = 1500;
int tim_delay = 30;
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

int Function13_value = 0;
int Function14_value = 0;

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
QUEUE *ftn_queue = new QUEUE[17];

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
  {62, 28},    //  Start Position Fx=0
  {63, 140},    //  End Position   Fx=1
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
  {85, 1}, //F11 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {86, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 1},   //  Start Position Fx=0
  {88, 5},  //  End Position   Fx=1
  {89, 1},    //  Current Position
  {90, 2}, //F12 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
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
  {105, 3}, //F15 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
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
  for (int i=8; i < numfpins; i++) {
     digitalWrite(fpins[i], 1);
     delay (tim_delay);
  }
  delay( tim_delay);
  for (int i=8; i < numfpins; i++) {
     digitalWrite(fpins[i], 0);
     delay (tim_delay);
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
      case 2:   //servo
       { 
         ftn_queue[i].current_position =int (Dcc.getCV( 34+(i*5)));
         ftn_queue[i].stop_value = int (Dcc.getCV( 33+(i*5)));
         ftn_queue[i].start_value = int (Dcc.getCV( 32+(i*5)));
         ftn_queue[i].increment = -int (char (Dcc.getCV( 31+(i*5)))); 
         // attaches servo on pin to the servo object 
         servo[i].attach(fpins[i]);

#ifdef DEBUG
	 Serial.print("InitServo ID= ");
	 Serial.println(i, DEC) ;
#endif
	 servo[i].write(ftn_queue[i].start_value);
         for (t=0; t<servo_start_delay; t++) 
		{SoftwareServo::refresh();delay(servo_init_delay);}
        ftn_queue[i].inuse = 0;
        servo[i].detach();
        }
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
  SoftwareServo::refresh();
  delay(2);
#ifdef DEBUG
   Serial.print("Motor1Speed= ");
   Serial.println(Motor1Speed, DEC) ;
   Serial.print("Motor2Speed= ");
   Serial.println(Motor2Speed, DEC) ;
#endif    
  if (Motor1Speed != 0) {
    if (Motor1ForwardDir == 0)  gofwd1 (fwdtime, Motor1Speed<<4);
      else gobwd1 (bwdtime, Motor1Speed<<4); 
  }
  else {
      digitalWrite(m2h, LOW);     //Motor1 OFF
      digitalWrite(m2l, LOW);     //Motor1 OFF
    }
  if (Motor2Speed != 0) {
    if (Motor2ForwardDir == 0)  gofwd2 (fwdtime, Motor2Speed<<4);
      else  gobwd2 (bwdtime, Motor2Speed<<4);
      }
    else {
    digitalWrite(m0h, LOW);     //Motor1 OFF
    digitalWrite(m0l, LOW);     //Motor1 OFF
   }
  //
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
        {
	  if (servo_slow_counter++ > servo_slowdown)
	    {
        ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
	    if (ftn_queue[i].increment > 0) {
	      if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
		ftn_queue[i].current_position = ftn_queue[i].stop_value;
                ftn_queue[i].inuse = 0;
		servo[i].detach();
              }
            }
	    if (ftn_queue[i].increment < 0) { 
	      if (ftn_queue[i].current_position < ftn_queue[i].start_value) { 
	        ftn_queue[i].current_position = ftn_queue[i].start_value;
                ftn_queue[i].inuse = 0;
		servo[i].detach();
              }
	    }
            servo[i].write(ftn_queue[i].current_position);
            servo_slow_counter = 0;
	    }
	   }
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
void gofwd1(int fcnt,int fcycle) {
   int icnt;
   int delta_tp,delta_tm;
   delta_tp = fcycle+loopdelay<<2;
   delta_tm = cyclewidth-fcycle-loopdelay;
   icnt = 0;
   while (icnt < fcnt)
  {
    digitalWrite(m2h, HIGH);     //Motor1
    delayMicroseconds(delta_tp);
    digitalWrite(m2h, LOW);     //Motor1
    delayMicroseconds(delta_tm);
    icnt++;
  }
}
void gobwd1(int bcnt,int bcycle) {
   int icnt;
   int delta_tp,delta_tm;
   delta_tp = bcycle+loopdelay<<2;
   delta_tm = cyclewidth-bcycle-loopdelay;
   icnt=0;
   while (icnt < bcnt)
  {
    digitalWrite(m2l, HIGH);     //Motor1
    delayMicroseconds(delta_tp); 
    digitalWrite(m2l, LOW);      //Motor1
    delayMicroseconds(delta_tm);
    icnt++;
  }
}
void gofwd2(int fcnt,int fcycle) {
   int icnt;
   int delta_tp,delta_tm;
   delta_tp = fcycle+loopdelay<<2;
   delta_tm = cyclewidth-fcycle-loopdelay;
   icnt = 0;
   while (icnt < fcnt)
  {
    digitalWrite(m0h, HIGH);     //Motor2
    delayMicroseconds(delta_tp);
    digitalWrite(m0h, LOW);     //Motor2
    delayMicroseconds(delta_tm);
    icnt++;
  }
}
void gobwd2(int bcnt,int bcycle) {
   int icnt;
   int delta_tp,delta_tm;
   delta_tp = bcycle+loopdelay<<2;
   delta_tm = cyclewidth-bcycle-loopdelay;
   icnt=0;
   while (icnt < bcnt)
  {
    digitalWrite(m0l, HIGH);     //Motor2
    delayMicroseconds(delta_tp); 
    digitalWrite(m0l, LOW);      //Motor2
    delayMicroseconds(delta_tm);
    icnt++;
  }
}
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION ForwardDir, DCC_SPEED_STEPS SpeedSteps )  {
   if (Function13_value==1)  {
     Motor1Speed = (Speed & 0x7f );
     if (Motor1Speed == 1)  Motor1Speed=0;
     Motor1ForwardDir  = ForwardDir;
   }
   if (Function14_value==1)  {
     Motor2Speed = (Speed & 0x7f );
     if (Motor2Speed == 1)  Motor2Speed=0;
     Motor2ForwardDir  = ForwardDir;
   }
}
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
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
      Function13_value = (FuncState & FN_BIT_13);
	  Function14_value = (FuncState & FN_BIT_14)>>1;
//	  exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
//	  exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
      break;
	
  case FN_21_28:
      break;	
  }
}  
void exec_function (int function, int pin, int FuncState)  {
  switch ( Dcc.getCV( 30+(function*5)) )  {  // Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
    case 0:    // On - Off LED
      digitalWrite (pin, FuncState);
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
      if (ftn_queue[function].inuse == 0)  {
	    ftn_queue[function].inuse = 1;
		servo[function].attach(pin);
	  }
      if (FuncState==1) ftn_queue[function].increment = char ( Dcc.getCV( 31+(function*5)));
        else ftn_queue[function].increment = - char(Dcc.getCV( 31+(function*5)));
      if (FuncState==1) ftn_queue[function].stop_value = Dcc.getCV( 33+(function*5));
        else ftn_queue[function].stop_value = Dcc.getCV( 32+(function*5));
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