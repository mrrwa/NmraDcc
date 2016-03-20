// Production Motor Drive 13 Pin Function DCC Decoder with Motor Drive
// F13 and F14 enable speed control of MOTOR1 and MOTOR2 respectively
// Version 5.0  Geoff Bunza 2015
// Uses modified software servo Lib
//
// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED
  
#include <NmraDcc.h>
#include <SoftwareServo.h> 

SoftwareServo servo0;
SoftwareServo servo1;
SoftwareServo servo2;
SoftwareServo servo3;
SoftwareServo servo4;
SoftwareServo servo5;
SoftwareServo servo6;
SoftwareServo servo7;
SoftwareServo servo8;
SoftwareServo servo9;
SoftwareServo servo10;
SoftwareServo servo11;
SoftwareServo servo12;
#define servo_start_delay 50
#define servo_init_delay 7

uint8_t Motor1Speed       = 0;
uint8_t Motor1ForwardDir  = 1;
uint8_t Motor1MaxSpeed    = 127;
uint8_t Motor2Speed       = 0;
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

int Function13_value = 0;
int Function14_value = 0;

NmraDcc  Dcc ;
DCC_MSG  Packet ;
uint8_t CV_DECODER_MASTER_RESET = 120;
int t;  // temp
#define This_Decoder_Address 24
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
CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},
  {CV_DECODER_MASTER_RESET, 0},
  {30, 0}, //F0 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {31, 1},    //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {32, 1},   //F0  Start Position F0=0
  {33, 1},  //F0  End Position   F0=1
  {34, 10},   //F0  Current Position
  {35, 0},  //F1 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {36, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {37, 1},   //  Start Position Fx=0
  {38, 1},  //  End Position   Fx=1
  {39, 10},  //  Current Position
  {40, 0},  //F2 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {41, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {42, 1},   //  Start Position Fx=0
  {43, 10},  //  End Position   Fx=1
  {44, 10},    //  Current Position
  {45, 0}, //F3 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {46, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {47, 1},   //  Start Position Fx=0
  {48, 1},  //  End Position   Fx=1
  {49, 10},    //  Current Position
  {50, 0}, //F4 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {51, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {52, 1},    //  Start Position Fx=0
  {53, 1},    //  End Position   Fx=1
  {54, 10},    //  Current Position
  {55, 0}, //F5 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {56, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {57, 1},    //  Start Position Fx=0
  {58, 1},    //  End Position   Fx=1
  {59, 10},    //  Current Position
  {60, 0}, //F6 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {61, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {62, 1},    //  Start Position Fx=0
  {63, 1},    //  End Position   Fx=1
  {64, 10},    //  Current Position
  {65, 0}, //F7 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {66, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {67, 1},   //  Start Position Fx=0
  {68, 1},  //  End Position   Fx=1
  {69, 1},    //  Current Position
  {70, 0}, //F8 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {71, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {72, 1},   //  Start Position Fx=0
  {73, 10},  //  End Position   Fx=1
  {74, 1},    //  Current Position
  {75, 0}, //F9 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {76, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {77, 1},   //  Start Position Fx=0
  {78, 10},  //  End Position   Fx=1
  {79, 1},    //  Current Position
  {80, 2}, //F10 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {81, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {82, 28},   //  Start Position Fx=0
  {83, 140},  //  End Position   Fx=1
  {84, 28},    //  Current Position
  {85, 0}, //F11 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {86, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 1},   //  Start Position Fx=0
  {88, 5},  //  End Position   Fx=1
  {89, 1},    //  Current Position
  {90, 0}, //F12 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {91, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {92, 1},   //  Start Position Fx=0
  {93, 1},  //  End Position   Fx=1
  {94, 28},    //  Current Position
  {95, 0}, //F13 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {96, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {97, 1},   //  Start Position Fx=0
  {98, 28},  //  End Position   Fx=1
  {99, 2},    //  Current Position
  {100, 0}, //F14 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {101, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {102, 1},   //  Start Position Fx=0
  {103, 4},  //  End Position   Fx=1
  {104, 1},    //  Current Position
  {105, 0}, //F15 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {106, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {107, 1},   //  Start Position Fx=0
  {108, 1},  //  End Position   Fx=1
  {109, 20},    //  Current Position
  {110, 0}, //F16 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {111, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {112, 1},   //  Start Position Fx=0
  {113, 1},  //  End Position   Fx=1
  {114, 1},    //  Current Position
//FUTURE USE
  {115, 0}, //F17 Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
  {116, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {117, 28},   //  Start Position Fx=0
  {118, 140},  //  End Position   Fx=1
  {119, 28},    //  Current Position
};

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

void setup()   //******************************************************
{
  int i;
  uint8_t cv_value;
  // initialize the digital pins as outputs
    for (int i=0; i < numfpins; i++) {
      pinMode(fpins[i], OUTPUT);
      digitalWrite(fpins[i], 0);
     }
  for (int i=0; i < numfpins; i++) {
     digitalWrite(fpins[i], 1);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  for (int i=0; i < numfpins; i++) {
     digitalWrite(fpins[i], 0);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  
  // Setup which External Interrupt, the Pin it's associated with that we're using 
  Dcc.pin(0, 2, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100, FLAGS_MY_ADDRESS_ONLY, 0 );
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
       { ftn_queue[i].current_position =int (Dcc.getCV( 34+(i*5)));
         ftn_queue[i].stop_value = int (Dcc.getCV( 33+(i*5)));
         ftn_queue[i].start_value = int (Dcc.getCV( 32+(i*5)));
         ftn_queue[i].increment = -int (char (Dcc.getCV( 31+(i*5)))); 
         switch ( i ) {
         case 0: servo0.attach(FunctionPin0);  // attaches servo on pin to the servo object 
           ftn_queue[i].inuse = 1;
           servo0.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 1:  servo1.attach(FunctionPin1);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo1.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 2: servo2.attach(FunctionPin2);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo2.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 3: servo3.attach(FunctionPin3);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo3.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 4: servo4.attach(FunctionPin4);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo4.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 5: servo5.attach(FunctionPin5);  // attaches servo on pin to the servo object  
           ftn_queue[i].inuse = 1;
           servo5.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 6: servo6.attach(FunctionPin6);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo6.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 7: servo7.attach(FunctionPin7);  // attaches servo on pin to the servo object  
           ftn_queue[i].inuse = 1;
           servo7.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 8: servo8.attach(FunctionPin8);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo8.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 9: servo9.attach(FunctionPin9);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo9.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 10: servo10.attach(FunctionPin10);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo10.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 11: servo11.attach(FunctionPin11);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo11.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         case 12: servo12.attach(FunctionPin12);  // attaches servo on pin to the servo object
           ftn_queue[i].inuse = 1;
           servo12.write(ftn_queue[i].start_value);
           for (t=0; t<servo_start_delay; t++) {SoftwareServo::refresh();delay(servo_init_delay);}
           break;
         default:
           break;
          }
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
       case 4:   // NEXT FEATURE to pin
         break;         
       default:
         break;
    }
  }
}

void loop()   //**********************************************************************
{
  boolean servo_on = true;
  //MUST call the NmraDcc.process() method frequently 
  // from the Arduino loop() function for correct library operation
  Dcc.process();
  SoftwareServo::refresh();
  delay(2);
  if (Motor1Speed != 0) {
    if (Motor1ForwardDir == 0)  gofwd1 (fwdtime, int((Motor1Speed&0x7f)*21));
    else gobwd1 (bwdtime, int((Motor1Speed&0x7f)*21));
  }
  if (Motor2Speed != 0) {
    if (Motor2ForwardDir == 0)  gofwd2 (fwdtime, int((Motor2Speed&0x7f)*21));
    else gobwd2 (bwdtime, int((Motor2Speed&0x7f)*21));
  }
  //
   for (int i=0; i < num_active_fpins; i++) {
    if (ftn_queue[i].inuse==1)  {
    ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
    servo_on = true;
    switch (Dcc.getCV( 30+(i*5))) {
      case 0:
        break;
      case 1:
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          ftn_queue[i].start_value = ~ftn_queue[i].start_value;
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
          ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5)));
        }
        break;
      case 2:
        {
        if (ftn_queue[i].increment > 0) {
          if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
            ftn_queue[i].current_position = ftn_queue[i].stop_value;
            servo_on = false;
            detach_servo (i);
          }
        } 
        if (ftn_queue[i].increment < 0) { 
          if (ftn_queue[i].current_position < ftn_queue[i].start_value) {
            ftn_queue[i].current_position = ftn_queue[i].start_value;
            servo_on = false;
            detach_servo (i);
          }
        }
        if (servo_on) {
          set_servo(i, ftn_queue[i].current_position);
          }
        }
        break;
      case 3:
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          ftn_queue[i].start_value = ~ftn_queue[i].start_value;
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          digitalWrite(fpins[i]+1, ~ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
          ftn_queue[i].stop_value = int(Dcc.getCV( 33+(i*5)));
        }
        i++;
        break;
      case 4:  //FUTURE FUNCTION
          break;
        default:
        break;  
      }
    }
    
  }
  //
}
void gofwd1(int fcnt,int fcycle) {
   int icnt;
   int totcycle;
   icnt = 0;
   while (icnt < fcnt)
  {
    digitalWrite(m2h, HIGH);     //Motor1
    delayMicroseconds(fcycle);
    digitalWrite(m2h, LOW);     //Motor1
    delayMicroseconds(cyclewidth - fcycle);
    icnt++;
  }
}
void gobwd1(int bcnt,int bcycle) {
   int icnt;
   icnt=0;
   while (icnt < bcnt)
  {
    digitalWrite(m2l, HIGH);     //Motor1
    delayMicroseconds(bcycle); 
    digitalWrite(m2l, LOW);      //Motor1
    delayMicroseconds(cyclewidth - bcycle);
    icnt++;
  }
}
void gofwd2(int fcnt,int fcycle) {
   int icnt;
   int totcycle;
   icnt = 0;
   while (icnt < fcnt)
  {
    digitalWrite(m0h, HIGH);     //Motor2
    delayMicroseconds(fcycle);
    digitalWrite(m0h, LOW);     //Motor2
    delayMicroseconds(cyclewidth - fcycle);
    icnt++;
  }
}
void gobwd2(int bcnt,int bcycle) {
   int icnt;
   icnt=0;
   while (icnt < bcnt)
  {
    digitalWrite(m0l, HIGH);     //Motor2
    delayMicroseconds(bcycle); 
    digitalWrite(m0l, LOW);      //Motor2
    delayMicroseconds(cyclewidth - bcycle);
    icnt++;
  }
}
void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )  {
   if (Function13_value==1)  {
     Motor1Speed = Speed;
     Motor1ForwardDir  = ForwardDir;
     Motor1MaxSpeed    = MaxSpeed;
   }
   if (Function14_value==1)  {
     Motor2Speed       = Speed;
     Motor2ForwardDir  = ForwardDir;
     Motor2MaxSpeed    = MaxSpeed;
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
/*     
	  exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
	  exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
*/
    break;
  case FN_21_28:
    break;	
  }
}  
void exec_function (int function, int pin, int FuncState)  {
  switch ( Dcc.getCV( 30+(function*5)) )  {  // Config  0=On/Off,1=Blink,2=Servo,3=Double LED Blink
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
      ftn_queue[function].inuse = 1;
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
    case 4:    // Future Function
      ftn_queue[function].inuse = 0;
      break;
    default:
      ftn_queue[function].inuse = 0;
      break;
  }
}
void set_servo (int servo_num, int servo_pos)  {
    switch (servo_num) {
         case 0: 
           if (servo0.attached()==0) servo0.attach(FunctionPin0);
           servo0.write(servo_pos);
           break;
         case 1: 
           if (servo1.attached()==0) servo1.attach(FunctionPin1);
           servo1.write(servo_pos);
           break;
         case 2: 
           if (servo2.attached()==0) servo2.attach(FunctionPin2);
           servo2.write(servo_pos);
           break;
         case 3: 
           if (servo3.attached()==0) servo3.attach(FunctionPin3);
           servo3.write(servo_pos);
           break;
         case 4: 
           if (servo4.attached()==0) servo4.attach(FunctionPin4);
           servo4.write(servo_pos);
           break;
         case 5: 
           if (servo5.attached()==0) servo5.attach(FunctionPin5);
           servo5.write(servo_pos);
           break;
         case 6: 
           if (servo6.attached()==0) servo6.attach(FunctionPin6);
           servo6.write(servo_pos);
           break;
         case 7: 
           if (servo7.attached()==0) servo7.attach(FunctionPin7);
           servo7.write(servo_pos);  
           break;
         case 8: 
           if (servo8.attached()==0) servo8.attach(FunctionPin8);
           servo8.write(servo_pos);
           break;
         case 9: 
           if (servo9.attached()==0) servo9.attach(FunctionPin9);
           servo9.write(servo_pos);
           break;
         case 10: 
           if (servo10.attached()==0) servo10.attach(FunctionPin10);
           servo10.write(servo_pos);
           break;
         case 11: 
           if (servo11.attached()==0) servo11.attach(FunctionPin11);
           servo11.write(servo_pos);
           break;
         case 12: 
           if (servo12.attached()==0) servo12.attach(FunctionPin12);
           servo12.write(servo_pos);
           break;
         default:
           break;
    }
}
void detach_servo (int servo_num)  {
    switch (servo_num) {
         case 0: 
           if (servo0.attached()!=0) servo0.detach();
           break;
         case 1: 
           if (servo1.attached()!=0) servo1.detach();
           break;
         case 2: 
           if (servo2.attached()!=0) servo2.detach();
           break;
         case 3: 
           if (servo3.attached()!=0) servo3.detach();
           break;
         case 4: 
           if (servo4.attached()!=0) servo4.detach();
           break;
         case 5: 
           if (servo5.attached()!=0) servo5.detach();
           break;
         case 6: 
           if (servo6.attached()!=0) servo6.detach();
           break;
         case 7: 
           if (servo7.attached()!=0) servo7.detach(); 
           break;
         case 8: 
           if (servo8.attached()!=0) servo8.detach();
           break;
         case 9: 
           if (servo9.attached()!=0) servo9.detach();
           break;
         case 10: 
           if (servo10.attached()!=0) servo10.detach();
           break;
         case 11: 
           if (servo11.attached()!=0) servo11.detach();
           break;
         case 12: 
           if (servo12.attached()!=0) servo12.detach();
         default:
           break;
    }
}

