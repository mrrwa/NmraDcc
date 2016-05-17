#include <NmraDcc.h>

// This Example shows how to use the library with the Iowa Scaled Engineering ARD-DCCSHIELD
// You can find out more about this DCC Interface here: http://www.iascaled.com/store/ARD-DCCSHIELD
//
// For more information refer to the file: README.md here: https://github.com/IowaScaledEngineering/ard-dccshield
//
// This demo assumes the following Jumper settings on the ARD-DCCSHIELD
//
// JP1 - I2C Pull-Up Resistors                        - Don't Care
// JP2 - (Pins 1-2) I2C /IORST JP2                    - Don't-Care
// JP2 - (Pins 3-4) - DCC Signal to Arduino Pin       - OFF
// JP3 - I2C /INT and /OE                             - Don't-Care
// JP4 - DCC Signal to Arduino Pin                    - D2 ON
// JP5 - Arduino Powered from DCC                     - User Choice
// JP6 - Boards without VIO                           - User Choice
// JP7 - Enable Programming ACK                       - 1-2 ON 3-4 ON 
// 
// It is a very basic DCC Accessory Decoder that does nothing except allow CV Read/Write and 
// you can also print every DCC packet by uncommenting the "#define NOTIFY_DCC_MSG" line below
#define NOTIFY_DCC_MSG

// You can also print other Debug Messages uncommenting the line below
#define DEBUG_MSG

// Un-Comment the line below to Enable DCC ACK for Service Mode Programming Read CV Capablilty 
#define ENABLE_DCC_ACK  15  // This is A1 on the Iowa Scaled Engineering ARD-DCCSHIELD DCC Shield

NmraDcc  Dcc ;

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
};

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
#ifdef DEBUG_MSG
  Serial.println("notifyCVResetFactoryDefault: Settings CVs to Factory Defaults") ;
#endif  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
#ifdef  ENABLE_DCC_ACK
void notifyCVAck(void)
{
#ifdef DEBUG_MSG
  Serial.println("notifyCVAck") ;
#endif
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( ENABLE_DCC_ACK, OUTPUT );

  // Generate the DCC ACK 60mA pulse
  digitalWrite( ENABLE_DCC_ACK, HIGH );
  delay( 10 );  // The DCC Spec says 6ms but 10 makes sure... ;)
  digitalWrite( ENABLE_DCC_ACK, LOW );
}
#endif

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

void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG_MSG
  Serial.print("notifyCVChange: CV: ") ;
  Serial.print(CV,DEC) ;
  Serial.print(" Value: ") ;
  Serial.println(Value, DEC) ;
#else
  Value = Value;  // Silence Compiler Warnings...
  CV = CV;
#endif  
}

void setup()
{
  Serial.begin(115200);
  
  Serial.println("NMRA DCC Iowa Scaled Engineering ARD-DCCSHIELD Example");
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER, 0 );

  Serial.println("Init Done");
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
