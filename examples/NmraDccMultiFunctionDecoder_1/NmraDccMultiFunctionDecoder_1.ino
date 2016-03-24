#include <NmraDcc.h>

#define This_Decoder_Address 3

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, This_Decoder_Address},

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, This_Decoder_Address},

// ONLY uncomment 1 CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
};

NmraDcc  Dcc ;

uint8_t FactoryDefaultCVIndex = 0;

// Uncomment this line below to force resetting the CVs back to Factory Defaults
// FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// Uncomment the #define below to print all Speed Packets
#define NOTIFY_DCC_SPEED
#ifdef  NOTIFY_DCC_SPEED
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(Speed,DEC);
  Serial.print(" Steps: ");
  Serial.print(SpeedSteps,DEC);
  Serial.print(" Dir: ");
  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
};
#endif

// Uncomment the #define below to print all Function Packets
#define NOTIFY_DCC_FUNC
#ifdef  NOTIFY_DCC_FUNC
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp,DEC);

  switch( FuncGrp )
   {
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE    
     case FN_0:
       Serial.print(" FN0: ");
       Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  "); 
       break;
#endif
       
     case FN_0_4:
       if(Dcc.getCV(CV_29_CONFIG) & CV29_F0_LOCATION) // Only process Function 0 in this packet if we're not in Speed Step 14 Mode
       {
         Serial.print(" FN 0: ");
         Serial.print((FuncState & FN_BIT_00) ? "1  ": "0  ");
       }
       
       Serial.print(" FN 1-4: ");
       Serial.print((FuncState & FN_BIT_01) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_02) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_03) ? "1  ": "0  ");
       Serial.println((FuncState & FN_BIT_04) ? "1  ": "0  ");
       break;
    
     case FN_5_8:
       Serial.print(" FN 5-8: ");
       Serial.print((FuncState & FN_BIT_05) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_06) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_07) ? "1  ": "0  ");
       Serial.println((FuncState & FN_BIT_08) ? "1  ": "0  ");
       break;
    
     case FN_9_12:
       Serial.print(" FN 9-12: ");
       Serial.print((FuncState & FN_BIT_09) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_10) ? "1  ": "0  ");
       Serial.print((FuncState & FN_BIT_11) ? "1  ": "0  ");
       Serial.println((FuncState & FN_BIT_12) ? "1  ": "0  ");
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
       break;  
   }
}
#endif

// Uncomment the #define below to print all DCC Packets
#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
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

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 

const int DccAckPin = 15 ;

void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;
  
  digitalWrite( DccAckPin, HIGH );
  delay( 8 );  
  digitalWrite( DccAckPin, LOW );
}

void setup()
{
  Serial.begin(115200);
  Serial.println("NMRA Dcc Multifunction Decoder Demo 1");

  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );
  digitalWrite( DccAckPin, LOW );
  
    // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 0);
  
  // Call the main DCC Init function to enable the DCC Receiver
  //Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY, 0 );

  // Uncomment to force CV Reset to Factory Defaults
  notifyCVResetFactoryDefault();
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}

