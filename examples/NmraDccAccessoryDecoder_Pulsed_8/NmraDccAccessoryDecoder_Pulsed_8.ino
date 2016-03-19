#include <NmraDcc.h>
#include "PinPulser.h"
// This Example shows how to use the library as a DCC Accessory Decoder to drive 8 Pulsed Turnouts

// You can print every DCC packet by un-commenting the line below
//#define NOTIFY_DCC_MSG

// You can print every notifyDccAccTurnoutOutput call-back by un-commenting the line below
#define NOTIFY_TURNOUT_MSG

// You can also print other Debug Messages uncommenting the line below
#define DEBUG_MSG

NmraDcc  Dcc ;
DCC_MSG  Packet ;

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

#define CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME 2  // CV for the Output Pulse ON ms
#define CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME 3  // CV for the delay in ms to allow a CDU to recharge
#define CV_ACCESSORY_DECODER_ACTIVE_STATE      4  // CV to define the ON Output State 

#define NUM_TURNOUTS 8                            // Number of Turnouts

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB,        1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB,        0},
  {CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME, 50},   // x 10mS for the output pulse duration
  {CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME, 30},   // x 10mS for the CDU recharge delay time
  {CV_ACCESSORY_DECODER_ACTIVE_STATE,    HIGH},
};

uint8_t FactoryDefaultCVIndex = 0;

// Un-Comment the line below to force CVs to be written to the Factory Default values defined above 
//FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs); 

// This is the Arduino Pin Mapping to Turnout Addresses with 2 pins per turnout 
//   base address  1  2  3  4  5  6   7   8   9  10  11  12  13  14  15  16
byte outputs[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
//   pins         D3 D4 D5 D6 D7 D8  D9 D10 D11 D12 D13  A0  A1  A2  A4  A5  

PinPulser pinPulser;

// To enable DCC CV Read capability with a DCC Service Mode CV Programmer un-comment the line below  
//#define ENABLE_DCC_ACK
#ifdef  ENABLE_DCC_ACK
const int DccAckPin = 3 ;
#endif

uint16_t BaseTurnoutAddress;  // 

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
#ifdef  NOTIFY_TURNOUT_MSG
  Serial.print("notifyDccAccTurnoutOutput: Turnout: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(" Direction: ");
  Serial.print(Direction ? "Closed" : "Thrown") ;
  Serial.print(" Output: ");
  Serial.print(OutputPower ? "On" : "Off") ;
#endif
  if(( Addr >= BaseTurnoutAddress ) && ( Addr < (BaseTurnoutAddress + NUM_TURNOUTS )) && OutputPower )
  {
    uint16_t pinIndex = ( (Addr - BaseTurnoutAddress) << 1 ) + Direction ;
    pinPulser.addPin(outputs[pinIndex]);
#ifdef  NOTIFY_TURNOUT_MSG
    Serial.print(" Pin Index: ");
    Serial.print(pinIndex,DEC);
    Serial.print(" Pin: ");
    Serial.print(outputs[pinIndex],DEC);
#endif
  }
#ifdef  NOTIFY_TURNOUT_MSG
  Serial.println();
#endif
}

void setup()
{
  Serial.begin(115200);
  
  // Configure the DCC CV Programing ACK pin for an output
#ifdef  ENABLE_DCC_ACK
  pinMode( DccAckPin, OUTPUT );
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  BaseTurnoutAddress = (Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) * 4) + Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) ; 
#ifdef DEBUG_MSG
  Serial.println("NMRA DCC 8-Turnout Accessory Decoder");
  Serial.print("DCC Turnout Base Address: ");
  Serial.println(BaseTurnoutAddress, DEC);
#endif  

  for(uint8_t i = 0; i < (NUM_TURNOUTS * 2); i++)
    pinMode( outputs[i], OUTPUT );

  uint16_t onMs              = Dcc.getCV(CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME) * 10;
  uint16_t cduRechargeMs     = Dcc.getCV(CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME) * 10;
  uint8_t  activeOutputState = Dcc.getCV(CV_ACCESSORY_DECODER_ACTIVE_STATE);

  pinPulser.init(onMs, cduRechargeMs, activeOutputState);

#ifdef DEBUG_MSG
  Serial.println("Init Done");
#endif  
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  
  pinPulser.process();
  
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}

void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG_MSG
  Serial.print("notifyCVChange: CV: ") ;
  Serial.print(CV,DEC) ;
  Serial.print(" Value: ") ;
  Serial.println(Value, DEC) ;
#endif  

  Value = Value;  // Silence Compiler Warnings...
  if((CV == CV_ACCESSORY_DECODER_ADDRESS_MSB) || (CV == CV_ACCESSORY_DECODER_ADDRESS_LSB))
  {
    BaseTurnoutAddress = (Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) * 4) + Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) ;
    return;
  }

  if((CV == CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME) || (CV == CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME) || (CV == CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME))
  {
    uint16_t onMs              = Dcc.getCV(CV_ACCESSORY_DECODER_OUTPUT_PULSE_TIME) * 10;
    uint16_t cduRechargeMs     = Dcc.getCV(CV_ACCESSORY_DECODER_CDU_RECHARGE_TIME) * 10;
    uint8_t  activeOutputState = Dcc.getCV(CV_ACCESSORY_DECODER_ACTIVE_STATE);

    pinPulser.init(onMs, cduRechargeMs, activeOutputState);
  }
}

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
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
  
  digitalWrite( DccAckPin, HIGH );
  delay( 6 );  
  digitalWrite( DccAckPin, LOW );
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
