// NMRA Dcc Multifunction Motor Decoder Demo using the Seeed XIAO Expansion board
// See: https://wiki.seeedstudio.com/Seeeduino-XIAO-Expansion-Board/
//
// Author: Alex Shepherd 2023-02-15
// 
// This example requires these Arduino Libraries:
//
// 1) The NmraDcc Library from: http://mrrwa.org/download/
//
// These libraries can be found and installed via the Arduino IDE Library Manager
//
// This simple demo displays the Multifunction Decoder actions on the builtin OLED Display
//

#include <NmraDcc.h>

#include <U8x8lib.h>
#include <Wire.h>

// Uncomment any of the lines below to enable debug messages for different parts of the code
#define DEBUG_FUNCTIONS
#define DEBUG_SPEED
//#define DEBUG_DCC_MSG

#if defined(DEBUG_FUNCTIONS) or defined(DEBUG_SPEED) or defined(DEBUG_PWM) or defined(DEBUG_DCC_MSG)
#define DEBUG_PRINT
#endif

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

#ifndef ARDUINO_SEEED_XIAO_M0
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif 

// I used a IoTT DCC Interface connected to Grove Analog Input which has A0 or 0 Pin
#define DCC_PIN     0

uint8_t newDirection = 0;
uint8_t lastDirection = 0;

uint8_t newSpeed = 0;
uint8_t lastSpeed = 0;
uint8_t numSpeedSteps = SPEED_STEP_128;

uint8_t lastFuncStateList[FN_LAST+1];

// Structure for CV Values Table
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// Default CV Values Table
CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(DEFAULT_DECODER_ADDRESS)},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(DEFAULT_DECODER_ADDRESS)},

// ONLY uncomment 1 CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
};

NmraDcc  Dcc ;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(PIN_WIRE_SCL, PIN_WIRE_SDA, U8X8_PIN_NONE);   // OLEDs without Reset of the Display

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  #ifdef DEBUG_SPEED
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(Speed,DEC);
  Serial.print(" Steps: ");
  Serial.print(SpeedSteps,DEC);
  Serial.print(" Dir: ");
  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  #endif

  newDirection = Dir;
  newSpeed = Speed;
  numSpeedSteps = SpeedSteps;
};

// This call-back function is called whenever we receive a DCC Function packet for our address 
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  #ifdef DEBUG_FUNCTIONS
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp,DEC);
  Serial.println();
  #endif

  if(lastFuncStateList[FuncGrp] != FuncState)
  {
    lastFuncStateList[FuncGrp] = FuncState;
    switch(FuncGrp)
    {
    #ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE    
      case FN_0:
        Serial.print(" FN0: ");
        Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  ");

        u8x8.setCursor(0, 2);
        u8x8.print("FN0 : ");
        u8x8.println((FuncState & FN_BIT_00) ? "1" : "0");
        break;
    #endif
       
      case FN_0_4:
        u8x8.setCursor(0, 2);
        u8x8.print("FN0 : ");
        
        if(Dcc.getCV(CV_29_CONFIG) & CV29_F0_LOCATION) // Only process Function 0 in this packet if we're not in Speed Step 14 Mode
        {
          Serial.print(" FN 0: ");
          Serial.print((FuncState & FN_BIT_00) ? "1  ": "0  ");
          
          u8x8.print((FuncState & FN_BIT_00) ? "1": "0");
        }
       
        Serial.print(" FN 1-4: ");
        Serial.print((FuncState & FN_BIT_01) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_02) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_03) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_04) ? "1  ": "0  ");

        u8x8.print((FuncState & FN_BIT_01) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_02) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_03) ? "1": "0");
        u8x8.println((FuncState & FN_BIT_04) ? "1": "0");
        break;
    
      case FN_5_8:
        Serial.print(" FN 5-8: ");
        Serial.print((FuncState & FN_BIT_05) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_06) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_07) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_08) ? "1  ": "0  ");

        u8x8.setCursor(0, 3);
        u8x8.print("FN5 : ");
        u8x8.print((FuncState & FN_BIT_05) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_06) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_07) ? "1": "0");
        u8x8.println((FuncState & FN_BIT_08) ? "1": "0");
        break;
    
      case FN_9_12:
        Serial.print(" FN 9-12: ");
        Serial.print((FuncState & FN_BIT_09) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_10) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_11) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_12) ? "1  ": "0  ");

        u8x8.setCursor(0, 4);
        u8x8.print("FN9 : ");
        u8x8.print((FuncState & FN_BIT_09) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_10) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_11) ? "1": "0");
        u8x8.println((FuncState & FN_BIT_12) ? "1": "0");
        break;
      
      case FN_13_20:
        Serial.print(" FN 13-20: ");
        Serial.print((FuncState & FN_BIT_13) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_14) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_15) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_16) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_17) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_18) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_19) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_20) ? "1  ": "0  ");

        u8x8.setCursor(0, 5);
        u8x8.print("FN13: ");
        u8x8.print((FuncState & FN_BIT_13) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_14) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_15) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_16) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_17) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_18) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_19) ? "1": "0");
        u8x8.println((FuncState & FN_BIT_20) ? "1": "0");
        break;
      
      case FN_21_28:
        Serial.print(" FN 21-28: ");
        Serial.print((FuncState & FN_BIT_21) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_22) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_23) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_24) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_25) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_26) ? "1  ": "0  ");
        Serial.print((FuncState & FN_BIT_27) ? "1  ": "0  ");
        Serial.println((FuncState & FN_BIT_28) ? "1  ": "0  ");

        u8x8.setCursor(0, 6);
        u8x8.print("FN21: ");
        u8x8.print((FuncState & FN_BIT_21) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_22) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_23) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_24) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_25) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_26) ? "1": "0");
        u8x8.print((FuncState & FN_BIT_27) ? "1": "0");
        u8x8.println((FuncState & FN_BIT_28) ? "1": "0");
        break;  
    }
  }
}

// This call-back function is called whenever we receive a DCC Packet
#ifdef  DEBUG_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

void setup()
{
  #ifdef DEBUG_PRINT
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);

  Serial.println("NMRA Dcc Multifunction Motor Decoder Demo");
  #endif

  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  u8x8.println("NMRA DCC");
  u8x8.println("MultiFunction");
  u8x8.println("Decoder Demo");
  delay(2000);
  u8x8.clearDisplay();
  u8x8.setCursor(0, 0);
  u8x8.println("Speed:");
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );

  // Uncomment to force CV Reset to Factory Defaults
//  notifyCVResetFactoryDefault();
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Handle Speed changes
  if((lastSpeed != newSpeed) || (lastDirection != newDirection))
  {
    lastSpeed = newSpeed;
    lastDirection = newDirection;

    u8x8.setCursor(0, 0);
    u8x8.print("Speed: ");
    u8x8.print(newSpeed);
    u8x8.print(":");
    u8x8.println( newDirection ? "Fwd" : "Rev");
  }
  
  // Handle resetting CVs back to Factory Defaults
  if( FactoryDefaultCVIndex )
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
