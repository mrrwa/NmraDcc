// This Example shows how to use the library as a DCC Accessory Decoder to drive 16 Servos to control Turnouts
#include <NmraDcc.h>
#include <Servo.h>
#include <SlowMotionServo.h>
#include <elapsedMillis.h>
#include <EEPROM.h>

// You can print every DCC packet by un-commenting the line below
//#define NOTIFY_DCC_MSG

// You can print every notifyDccAccTurnoutOutput call-back by un-commenting the line below
#define NOTIFY_TURNOUT_MSG

// You can also print other Debug Messages uncommenting the line below
#define DEBUG_MSG

// Un-Comment the line below to force CVs to be written to the Factory Default values
// defined in the FactoryDefaultCVs below on Start-Up
//#define FORCE_RESET_FACTORY_DEFAULT_CV

// Un-Comment the line below to Enable DCC ACK for Service Mode Programming Read CV Capablilty 
#define ENABLE_DCC_ACK  27  // This is A1 on the Iowa Scaled Engineering ARD-DCCSHIELD DCC Shield

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN     26

#define NUM_SERVOS  16              // Set Number of Servos

#define DCC_DECODER_VERSION_NUM 12  // Set the Decoder Version - Used by JMRI to Identify the decoder

#define EEPROM_COMMIT_DELAY_MS  3000

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

#define CV_VALUE_SERVO_DETACH_MILLIS   0  // CV Default Value for the delay in ms x 10ms to Detach the servo signal to stop chattering
#define CV_ADDRESS_SERVO_DETACH_MILLIS CV_MANUFACTURER_START

#define CV_VALUE_SERVO_MOVE_SPEED     40  // CV Default Value for the Serov movement speed x 10
#define CV_ADDRESS_SERVO_MOVE_SPEED   (CV_MANUFACTURER_START + 1)

#define CV_ADDRESS_SERVO_INDIVIDUAL_VALUES (CV_MANUFACTURER_START + 2)

#define CV_ADDRESS_LAST_POS_LSB       (CV_MANUFACTURER_START + (NUM_SERVOS * 2 ) + 3)
#define CV_ADDRESS_LAST_POS_MSB       (CV_MANUFACTURER_START + (NUM_SERVOS * 2 ) + 4)

#define CV_VALUE_SERVO_MIN_POS        40  // CV Default Value for the Minimum Servo Position as a % of Full Scale
#define CV_ADDRESS_SERVO_MIN_POS(x) (CV_ADDRESS_SERVO_INDIVIDUAL_VALUES + (2 * x) + 1)

#define CV_VALUE_SERVO_MAX_POS        60  // CV Default Value for the Maximum Servo Position as a % of Full Scale
#define CV_ADDRESS_SERVO_MAX_POS(x) (CV_ADDRESS_SERVO_INDIVIDUAL_VALUES + (2 * x) + 2)

// To set the Turnout Addresses for this board you need to change the CV values for CV1 (CV_ACCESSORY_DECODER_ADDRESS_LSB) and 
// CV9 (CV_ACCESSORY_DECODER_ADDRESS_MSB) in the FactoryDefaultCVs structure below. The Turnout Addresses are defined as: 
// Base Turnout Address is: ((((CV9 * 64) + CV1) - 1) * 4) + 1 
// With NUM_TURNOUTS 8 (above) a CV1 = 1 and CV9 = 0, the Turnout Addresses will be 1..8, for CV1 = 2 the Turnout Address is 5..12

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8},
  {CV_ADDRESS_SERVO_DETACH_MILLIS, CV_VALUE_SERVO_DETACH_MILLIS},  // 0 = don't Detach else the Detach will occur after the CV Value x 10ms
  {CV_ADDRESS_SERVO_MOVE_SPEED, CV_VALUE_SERVO_MOVE_SPEED},  // 0 = don't Detach else the Detach will occur after the CV Value x 10ms

  {CV_ADDRESS_SERVO_MIN_POS(0), CV_VALUE_SERVO_MIN_POS},    // Servo 0
  {CV_ADDRESS_SERVO_MAX_POS(0), CV_VALUE_SERVO_MAX_POS},
  
  {CV_ADDRESS_SERVO_MIN_POS(1), CV_VALUE_SERVO_MIN_POS},    // Servo 1
  {CV_ADDRESS_SERVO_MAX_POS(1), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(2), CV_VALUE_SERVO_MIN_POS},    // Servo 2
  {CV_ADDRESS_SERVO_MAX_POS(2), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(3), CV_VALUE_SERVO_MIN_POS},    // Servo 3
  {CV_ADDRESS_SERVO_MAX_POS(3), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(4), CV_VALUE_SERVO_MIN_POS},    // Servo 4
  {CV_ADDRESS_SERVO_MAX_POS(4), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(5), CV_VALUE_SERVO_MIN_POS},    // Servo 5
  {CV_ADDRESS_SERVO_MAX_POS(5), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(6), CV_VALUE_SERVO_MIN_POS},    // Servo 6
  {CV_ADDRESS_SERVO_MAX_POS(6), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(7), CV_VALUE_SERVO_MIN_POS},    // Servo 7
  {CV_ADDRESS_SERVO_MAX_POS(7), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(8), CV_VALUE_SERVO_MIN_POS},    // Servo 8
  {CV_ADDRESS_SERVO_MAX_POS(8), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(9), CV_VALUE_SERVO_MIN_POS},    // Servo 9
  {CV_ADDRESS_SERVO_MAX_POS(9), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(10), CV_VALUE_SERVO_MIN_POS},    // Servo 10
  {CV_ADDRESS_SERVO_MAX_POS(10), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(11), CV_VALUE_SERVO_MIN_POS},    // Servo 11
  {CV_ADDRESS_SERVO_MAX_POS(11), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(12), CV_VALUE_SERVO_MIN_POS},    // Servo 12
  {CV_ADDRESS_SERVO_MAX_POS(12), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(13), CV_VALUE_SERVO_MIN_POS},    // Servo 13
  {CV_ADDRESS_SERVO_MAX_POS(13), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(14), CV_VALUE_SERVO_MIN_POS},    // Servo 14
  {CV_ADDRESS_SERVO_MAX_POS(14), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_SERVO_MIN_POS(15), CV_VALUE_SERVO_MIN_POS},    // Servo 15
  {CV_ADDRESS_SERVO_MAX_POS(15), CV_VALUE_SERVO_MAX_POS},

  {CV_ADDRESS_LAST_POS_LSB,     0},
  {CV_ADDRESS_LAST_POS_MSB,     0},
};

uint8_t FactoryDefaultCVIndex = 0;

NmraDcc  Dcc ;
DCC_MSG  Packet ;
uint16_t BaseTurnoutAddress;

uint16_t lastPositionBits;

SMSSmooth Servos[NUM_SERVOS];

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput (uint16_t OutputAddress, uint8_t Direction, uint8_t OutputPower)
{
#ifdef  NOTIFY_TURNOUT_MSG
  Serial.print("notifyDccAccTurnoutOutput: Output Addr: ") ;
  Serial.print(OutputAddress,DEC) ;
  Serial.print(" Direction: ");
  Serial.print(Direction ? "Closed" : "Thrown") ;
  Serial.print(" Output: ");
  Serial.print(OutputPower ? "On" : "Off") ;
#endif
  if(( OutputAddress >= BaseTurnoutAddress ) && ( OutputAddress < (BaseTurnoutAddress + NUM_SERVOS )) && OutputPower ) // Only Drive the Servo on Activation  
  {
    uint16_t servoIndex = OutputAddress - BaseTurnoutAddress;
    
#ifdef  NOTIFY_TURNOUT_MSG
    Serial.print(" Servo Num: ");
    Serial.println(servoIndex,DEC);
#endif

    setServoPos(servoIndex, Direction != 0);
  }
  else
  {
#ifdef  NOTIFY_TURNOUT_MSG
    Serial.println();
#endif
  }


}

void notifyDccAccTurnoutBoard (uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower)
{
uint16_t Addr = ((BoardAddr - 1) * 4) + OutputPair + 1;

#ifdef  NOTIFY_TURNOUT_MSG
  Serial.print("notifyDccAccTurnoutBoard: Board Addr: ") ;
  Serial.print(BoardAddr,DEC) ;
  Serial.print(" Output Pair: ");
  Serial.print(OutputPair,DEC) ;
  Serial.print(" Turnout Addr: ");
  Serial.print(Addr,DEC) ;
  Serial.print(" Base Turnout Addr: ");
  Serial.print(BaseTurnoutAddress,DEC) ;
  Serial.print(" Direction: ");
  Serial.print(Direction ? "Closed" : "Thrown") ;
  Serial.print(" Output: ");
  Serial.print(OutputPower ? "On" : "Off") ;
  
#endif

  if(( Addr >= BaseTurnoutAddress ) && ( Addr < (BaseTurnoutAddress + NUM_SERVOS )) && OutputPower ) // Only Drive the Servo on Activation  
  {
    uint16_t servoIndex = Addr - BaseTurnoutAddress;
    
#ifdef  NOTIFY_TURNOUT_MSG
    Serial.print(" Servo Num: ");
    Serial.println(servoIndex,DEC);
#endif

    setServoPos(servoIndex, Direction != 0);
  }
  else
  {
#ifdef  NOTIFY_TURNOUT_MSG
    Serial.println();
#endif
  }
}


void setServoPos(uint8_t index, bool position)
{
  if(index < NUM_SERVOS)
  {
    Servos[index].goTo( position ? 1.0 : 0.0);
    
    setLastTurnoutPosition(index, position);
  }
}

bool getLastTurnoutPosition(uint8_t index)
{
  if(index < NUM_SERVOS)
  {
    uint16_t bitMask = 1 << index;
    return lastPositionBits & bitMask ? true : false;
  }
  return 0;
}

void setLastTurnoutPosition(uint8_t index, bool position)
{
  if(index < NUM_SERVOS)
  {
    uint16_t bitMask = 1 << index;

    if(position)
      lastPositionBits |= bitMask;
    else    
      lastPositionBits &= ~bitMask;

    Dcc.setCV(CV_ADDRESS_LAST_POS_LSB, lastPositionBits & 0x00ff);
    Dcc.setCV(CV_ADDRESS_LAST_POS_MSB, (lastPositionBits >> 8) & 0x00ff);
  }
}

void initServos()
{
  BaseTurnoutAddress = (((Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) * 64) + Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) - 1) * 4) + 1  ;

  uint16_t servoDetachMillis = Dcc.getCV(CV_ADDRESS_SERVO_DETACH_MILLIS) * 10;
  SlowMotionServo::setDelayUntilStop(servoDetachMillis); // This is a static class member so only needs to be set once and is used globally 

#ifdef DEBUG_MSG
  Serial.print("initServos: DCC Turnout Base Address: "); Serial.print(BaseTurnoutAddress, DEC);
  Serial.print("  Detach Servo Signal MilliSeconds: "); Serial.print(servoDetachMillis);  
#endif  

  float servoMoveSpeed = Dcc.getCV(CV_ADDRESS_SERVO_MOVE_SPEED) / 10.0;
  lastPositionBits = Dcc.getCV(CV_ADDRESS_LAST_POS_LSB) | (Dcc.getCV(CV_ADDRESS_LAST_POS_MSB) << 8);

  for(uint8_t i = 0; i < NUM_SERVOS; i++)
  {
    Servos[i].setPin(i);
    Servos[i].setupMin( map(Dcc.getCV(CV_ADDRESS_SERVO_MIN_POS(i)), 0, 100, 544, 2400));
    Servos[i].setupMax( map(Dcc.getCV(CV_ADDRESS_SERVO_MAX_POS(i)), 0, 100, 544, 2400));
    Servos[i].setSpeed(servoMoveSpeed);
    Servos[i].setDetach(servoDetachMillis != 0);
    Servos[i].setInitialPosition(getLastTurnoutPosition(i) ? 1.0 : 0.0);
  }
  
  SlowMotionServo::update();
  
#ifdef DEBUG_MSG
  Serial.println("\ninitServos: Done");
#endif  
}  


void setup()
{
  Serial.begin(115200);

  elapsedMillis millisWaitedForUSB = 0;
  while(!Serial && (millisWaitedForUSB < 3000));  // Wait up to 3 seconds for USB to Connect

#ifdef DEBUG_MSG
  Serial.print("\nNMRA DCC 16-Servo Accessory Decoder. Ver: "); Serial.println(DCC_DECODER_VERSION_NUM,DEC);
#endif  

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 1);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_NUM, FLAGS_DCC_ACCESSORY_DECODER|FLAGS_EXTENDED_ADDRESS_MODE, 0);

#ifdef FORCE_RESET_FACTORY_DEFAULT_CV
  Serial.println("Resetting CVs to Factory Defaults");
  notifyCVResetFactoryDefault(); 
#endif

  if( FactoryDefaultCVIndex == 0)	// Not forcing a reset CV Reset to Factory Defaults so initPinPulser
	  initServos();  
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    uint16_t cv = FactoryDefaultCVs[FactoryDefaultCVIndex].CV;
    uint8_t val = FactoryDefaultCVs[FactoryDefaultCVIndex].Value;
#ifdef DEBUG_MSG
    Serial.print("loop: Write Default CV: "); Serial.print(cv,DEC); Serial.print(" Value: "); Serial.println(val,DEC);
#endif     
    Dcc.setCV( cv, val );
    
    if( FactoryDefaultCVIndex == 0)	// Is this the last Default CV to set? if so re-initPinPulser
	    initServos();
  }
  
  SlowMotionServo::update();
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

  if((CV == CV_ACCESSORY_DECODER_ADDRESS_MSB) || (CV == CV_ACCESSORY_DECODER_ADDRESS_LSB) ||
      ((CV >= CV_ADDRESS_SERVO_DETACH_MILLIS) && (CV < CV_ADDRESS_LAST_POS_LSB)) && (FactoryDefaultCVIndex == 0))
		initServos();	// Some CV we care about changed so re-init the PinPulser with the new CV settings
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
