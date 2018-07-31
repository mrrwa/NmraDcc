// Production 17 Switch Acessory DCC Decoder    AccDec_7ServoBackandForth6Ftn.ino
// Version 6.01  Geoff Bunza 2014,2015,2016,2017,2018
// Now works with both short and long DCC Addesses for CV Control Default 24 (LSB CV 121 ; MSB CV 122)
// ACCESSORY DECODER  DEFAULT ADDRESS IS 40 (MAX 40-56 SWITCHES)
// ACCESSRY DECODER ADDRESS CAN NOW BE SET ABOVE 255
// BE CAREFUL!  DIFFERENT DCC BASE STATIONS  ALLOW DIFFERING MAX ADDRESSES

// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks

// This Decoder Version has been modified so that each Switch Closure Transition from Thrown to Closed
//  Swings the Servo Quickly from Start to Stop and Back to Start
//  This is ONLY done in the transition from Thrown to Closed Servo Speed can be slowed by changing the 
//  RATE CV towards 1

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
//#define DECODER_LOADED

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
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
const int FunctionPin14 = 17;     //A3
const int FunctionPin15 = 18;     //A4
const int FunctionPin16 = 19;     //A5
NmraDcc  Dcc ;
DCC_MSG  Packet ;

int t;                                    // temp
#define SET_CV_Address       24           // THIS ADDRESS IS FOR SETTING CV'S Like a Loco
#define Accessory_Address    40           // THIS ADDRESS IS THE START OF THE SWITCHES RANGE
                                          // WHICH WILL EXTEND FOR 16 MORE SWITCH ADDRESSES
uint8_t CV_DECODER_MASTER_RESET =   120;  // THIS IS THE CV ADDRESS OF THE FULL RESET
#define CV_To_Store_SET_CV_Address	121
#define CV_Accessory_Address CV_ACCESSORY_DECODER_ADDRESS_LSB
						  
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
  // These two CVs define the Long Accessory Address
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, Accessory_Address&0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, (Accessory_Address>>8)&0x07},
  
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},

  // Speed Steps don't matter for this decoder
  // ONLY uncomment 1 CV_29_CONFIG line below as approprate DEFAULT IS SHORT ADDRESS
  //  {CV_29_CONFIG,          0},                                           // Short Address 14 Speed Steps
  //  {CV_29_CONFIG, CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
  //  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},   // Long  Address 28/128 Speed Steps  
  {CV_29_CONFIG,CV29_ACCESSORY_DECODER|CV29_OUTPUT_ADDRESS_MODE|CV29_F0_LOCATION}, // Accesory Decoder Short Address
  //  {CV_29_CONFIG, CV29_ACCESSORY_DECODER|CV29_OUTPUT_ADDRESS_MODE|CV29_EXT_ADDRESSING | CV29_F0_LOCATION},  // Accesory Decoder  Long  Address 

  {CV_DECODER_MASTER_RESET, 0},
  {CV_To_Store_SET_CV_Address, SET_CV_Address&0xFF },   // LSB Set CV Address
  {CV_To_Store_SET_CV_Address+1,(SET_CV_Address>>8)&0x3F },  //MSB Set CV Address
  {30, 2}, //F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {31, 1},    //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {32, 28},   //F0  Start Position 
  {33, 140},  //F0  End Position  
  {34, 28},   //F0  Current Position
  {35, 2},  //F1 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {36, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {37, 28},   //  Start Position 
  {38, 140},  //  End Position  
  {39, 28},  //  Current Position
  {40, 2},  //F2 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {41, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {42, 28},   //  Start Position
  {43, 140},  //  End Position 
  {44, 28},    //  Current Position
  {45, 2}, //F3 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {46, 2},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {47, 28},   //  Start Position
  {48, 140},  //  End Position  
  {49, 28},    //  Current Position
  {50, 2}, //F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {51, 2},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {52, 28},    //  Start Position 
  {53, 140},  //  End Position  
  {54, 28},    //  Current Position
  {55, 2}, //F5 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {56, 3},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {57, 28},    //  Start Position 
  {58, 140},   //  End Position   
  {59, 28},    //  Current Position
  {60, 2}, //F6 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {61, 3},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {62, 28},    //  Start Position 
  {63, 140},   //  End Position  
  {64, 28},    //  Current Position
  {65, 1}, //F7 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {66, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {67, 28}, //  Start Position 
  {68, 140},  //  End Position   
  {69, 1},   //  Current Position
  {70, 1}, //F8 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {71, 1},   // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {72, 1},   //  Start Position 
  {73, 100}, //  End Position  
  {74, 1},   //  Current Position
  {75, 0}, //F9 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {76, 1},   // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {77, 1},   //  Start Position 
  {78, 10},  //  End Position 
  {79, 1},   //  Current Position
  {80, 0}, //F10 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {81, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {82, 1},  //  Start Position 
  {83, 5},  //  End Position   
  {84, 1},  //  Current Position
  {85, 1}, //F11 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {86, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 1},  //  Start Position 
  {88, 5},  //  End Position   
  {89, 1},  //  Current Position
  {90, 1}, //F12 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {91, 1},   // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {92, 1},   //  Start Position 
  {93, 20},  //  End Position 
  {94, 1},   //  Current Position
  {95, 1}, //F13 Config  0=On/Off,1=Blink,2=Servo,3=PWM
  {96, 1},   // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {97, 1},   //  Start Position 
  {98, 20},  //  End Position   
  {99, 1},   //  Current Position
  {100, 0}, //F14 Config  0=On/Off,1=Blink,2=Servo,3=PWM
  {101, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {102, 1},  //  Start Position 
  {103, 4},  //  End Position   
  {104, 1},  //  Current Position
  {105, 3}, //F15 Config  0=On/Off,1=Blink,2=Servo,3=PWM
  {106, 1},   // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {107, 1},   //  Start Position 
  {108, 60},  //  End Position  
  {109, 20},  //  Current Position
  {110, 0}, //F16 Config  0=On/Off,1=Blink,2=Servo,3=PWM
  {111, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {112, 1},  //  Start Position
  {113, 4},  //  End Position   
  {114, 1},  //  Current Position
//FUTURE USE
  {115, 0}, //F17 Config  0=On/Off,1=Blink,2=Servo,3=PWM
  {116, 1},  // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {117, 28}, //  Start Position 
  {118, 50}, //  End Position   
  {119, 28}, //  Current Position
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
  Dcc.init( MAN_ID_DIY, 601, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, CV_To_Store_SET_CV_Address);
  delay(800);
   
  #if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET ) 
  #endif  
  
     {
       for (int j=0; j < sizeof(FactoryDefaultCVs)/sizeof(CVPair); j++ )
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
      case 2:  // All Servo service timing is now local to the Turn on transition
        
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

extern void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
  uint16_t Current_Decoder_Addr = Dcc.getAddr();
  
  if ( Addr >= Current_Decoder_Addr && Addr < Current_Decoder_Addr+17) { //Controls Accessory_Address+16
#ifdef DEBUG
	 Serial.print("Addr = ");
	 Serial.println(Addr);
	 Serial.print("Direction = ");
	 Serial.println(Direction);
#endif
	exec_function(Addr-Current_Decoder_Addr, Direction );
  } 
}

void exec_function (int function, int FuncState)  {
  byte pin;
  int servo_temp;
  pin = fpins[function];
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
    case 2: {   // Servo
      if ((ftn_queue[function].inuse == 0) && (FuncState==1)) {  // We have an OFF->ON transition
	    servo[function].attach(pin);
		ftn_queue[function].increment = char ( Dcc.getCV( 31+(function*5)));
	    ftn_queue[function].start_value = Dcc.getCV( 32+(function*5));
            ftn_queue[function].stop_value = Dcc.getCV( 33+(function*5));
	    for (servo_temp=ftn_queue[function].start_value; servo_temp<ftn_queue[function].stop_value; servo_temp=servo_temp+ftn_queue[function].increment) {
		  servo[function].write(servo_temp);
		  SoftwareServo::refresh();
		  delay(4);
		  }
		for (servo_temp=ftn_queue[function].stop_value; servo_temp>ftn_queue[function].start_value; servo_temp=servo_temp-ftn_queue[function].increment) {
		  servo[function].write(servo_temp);
		  SoftwareServo::refresh();
		  delay(4);
		  }
      ftn_queue[function].inuse = 1;
	  }
	  if (FuncState==0) {
	    ftn_queue[function].inuse = 0;
	    servo[function].detach();
	  }
      break;
    }
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
