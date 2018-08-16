// Production 17 Function DCC Decoder    Dec_13Serv_4LED_6Ftn.ino
// Version 6.01  Geoff Bunza 2014,2015,2016
// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** EMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

#include <NmraDcc.h>
#include <SoftwareServo.h> 

SoftwareServo servo[17];
#define servo_start_delay 50
#define servo_init_delay 7
#define servo_slowdown  12   //servo loop counter limit
int servo_slow_counter = 0; //servo loop counter to slowdown servo transit

int tim_delay = 500;
int numfpins = 17;
byte fpins [] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
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
const int FunctionPin14 = 17;     //A3 & LOAD ACK
const int FunctionPin15 = 18;     //A4
const int FunctionPin16 = 19;     //A5
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

  {CV_DECODER_MASTER_RESET, 0},
  {30, 2}, //F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {31, 1},    //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {32, 28},   //F0  Start Position F0=0
  {33, 140},  //F0  End Position   F0=1
  {34, 28},   //F0  Current Position
  {35, 2},  //F1 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {36, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {37, 28},   //  Start Position Fx=0
  {38, 140},  //  End Position   Fx=1
  {39, 28},  //  Current Position
  {40, 2},  //F2 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {41, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {42, 28},   //  Start Position Fx=0
  {43, 140},  //  End Position   Fx=1
  {44, 28},    //  Current Position
  {45, 2}, //F3 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {46, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {47, 28},   //  Start Position Fx=0
  {48, 140},  //  End Position   Fx=1
  {49, 28},    //  Current Position
  {50, 2}, //F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {51, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {52, 28},    //  Start Position Fx=0
  {53, 140},    //  End Position   Fx=1
  {54, 28},    //  Current Position
  {55, 2}, //F5 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {56, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {57, 28},    //  Start Position Fx=0
  {58, 140},    //  End Position   Fx=1
  {59, 28},    //  Current Position
  {60, 2}, //F6 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {61, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {62, 28},    //  Start Position Fx=0
  {63, 140},    //  End Position   Fx=1
  {64, 28},    //  Current Position
  {65, 2}, //F7 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {66, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {67, 28},   //  Start Position Fx=0
  {68, 140},  //  End Position   Fx=1
  {69, 28},    //  Current Position
  {70, 2}, //F8 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {71, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {72, 28},   //  Start Position Fx=0
  {73, 140},  //  End Position   Fx=1
  {74, 28},    //  Current Position
  {75, 2}, //F9 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {76, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {77, 28},   //  Start Position Fx=0
  {78, 140},  //  End Position   Fx=1
  {79, 28},    //  Current Position
  {80, 2}, //F10 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {81, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {82, 28},   //  Start Position Fx=0
  {83, 140},  //  End Position   Fx=1
  {84, 28},    //  Current Position
  {85, 2}, //F11 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {86, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 28},   //  Start Position Fx=0
  {88, 140},  //  End Position   Fx=1
  {89, 28},    //  Current Position
  {90, 2}, //F12 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {91, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {92, 28},   //  Start Position Fx=0
  {93, 140},  //  End Position   Fx=1
  {94, 28},    //  Current Position
  {95, 3}, //F13 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {96, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {97, 1},   //  Start Position Fx=0
  {98, 200},  //  End Position   Fx=1
  {99, 2},    //  Current Position
  {100, 0}, //F14 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {101, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {102, 1},   //  Start Position Fx=0
  {103, 200},  //  End Position   Fx=1
  {104, 1},    //  Current Position
  {105, 3}, //F15 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {106, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {107, 1},   //  Start Position Fx=0
  {108, 60},  //  End Position   Fx=1
  {109, 1},    //  Current Position
  {110, 0}, //F16 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {111, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {112, 1},   //  Start Position Fx=0
  {113, 4},  //  End Position   Fx=1
  {114, 1},    //  Current Position
//FUTURE USE
  {115, 0}, //F17 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
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
  for ( i=0; i < numfpins; i++) {
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
  delay(3);
  for (int i=0; i < numfpins; i++) {
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
      exec_function( 13, FunctionPin13, (FuncState & FN_BIT_13));
	  exec_function( 14, FunctionPin14, (FuncState & FN_BIT_14)>>1 );
	  exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
	  exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
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
