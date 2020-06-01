// DCC Stepper Motor Controller ( A4988 ) Example for Model Railroad Turntable Control
//
// See: https://www.dccinterface.com/product/arduino-model-railway-dcc-stepper-motor-controller-a4988-assembled/
// 
// Author: Alex Shepherd 2020-06-01
// 
// This example requires two Arduino Libraries:
//
// 1) The AccelStepper library from: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//
// 2) The NmraDcc Library from: http://mrrwa.org/download/
//
// Both libraries can be found and installed via the Arduino IDE Library Manager
//
// Also checkout the artical I wrote in this project here:
//         http://mrrwa.org/2017/12/23/dcc-controlled-turntable-stepper-motor-driver/ 
//

#include <AccelStepper.h>
#include <NmraDcc.h>

// The lines below define the pins used to connect to the A4988 driver module
#define A4988_STEP_PIN      4
#define A4988_DIRECTION_PIN 5
#define A4988_ENABLE_PIN    6

#ifdef A4988_ENABLE_PIN
// Uncomment the next line to enable Powering-Off the Stepper when its not running to reduce heating the motor and driver
#define DISABLE_OUTPUTS_IDLE
#endif

// By default the stepper motor will move the shortest distance to the desired position.
// If you need the turntable to only move in the Positive/Increasing or Negative/Decreasing step numbers to better handle backlash in the mechanism
// Then uncomment the appropriate line below
//#define ALWAYS_MOVE_POSITIVE
//#define ALWAYS_MOVE_NEGATIVE

// The lines below define the stepping speed and acceleration, which you may need to tune for your application
#define STEPPER_MAX_SPEED     800   // Sets the maximum permitted speed
#define STEPPER_ACCELARATION  1000  // Sets the acceleration/deceleration rate
#define STEPPER_SPEED         300   // Sets the desired constant speed for use with runSpeed()

// The line below defines the number of "Full Steps" your stepper motor does for a full rotation
#define MOTOR_FULL_STEPS_PER_REVOLUTION 200

// The line below defines any reduction gearbox multiplier. No gearbox = 1 
#define REDUCTION_GEARBOX_RATIO 1

#define STEPS_PER_REVOLUTION (MOTOR_FULL_STEPS_PER_REVOLUTION * REDUCTION_GEARBOX_RATIO)

// The A4988 Driver Board has 3 pins that set the Stepping Mode which are connected to 3 jumpers on the board. 
// Uncomment the line below to match the Boards jumper setting        MS1,     MS2,     MS3
// --------------------------------------------------------------------------------------------
//#define FULL_TURN_STEPS (STEPS_PER_REVOLUTION)      // full steps - MS1=OFF, MS2=OFF, MS3=OFF
//#define FULL_TURN_STEPS (STEPS_PER_REVOLUTION * 2)  // 1/2  steps - MS1=ON,  MS2=OFF, MS3=OFF
#define FULL_TURN_STEPS (STEPS_PER_REVOLUTION * 4)  // 1/4  steps - MS1=OFF, MS2=ON,  MS3=OFF
//#define FULL_TURN_STEPS (STEPS_PER_REVOLUTION * 8)  // 1/8  steps - MS1=ON,  MS2=ON,  MS3=OFF
//#define FULL_TURN_STEPS (STEPS_PER_REVOLUTION * 16) // 1/16 steps - MS1=ON,  MS2=ON,  MS3=ON

#ifndef FULL_TURN_STEPS
#error You need to select one of the FULL_TURN_STEPS to match the A4988 Driver Board jumper settings
#endif

// This constant is useful to know the number of steps to rotate the turntable 180 degrees for the back entrance position
#define HALF_TURN_STEPS (FULL_TURN_STEPS / 2)

// Home Position Sensor Input
#define HOME_SENSOR_PIN 3
#define HOME_SENSOR_ACTIVE_STATE HIGH

// This structure holds the values for a turntable position wiht the DCC Address, Front Position in Steps from Home Sensor
typedef struct
{
  int dccAddress;
  int positionFront;
  int positionBack;
}
TurnoutPosition;

// The constant HOME_POSITION_DCC_ADDRESS is the base DCC Accessory Decoder Address for the Home Position
// with each subsequent position numbered sequentially from there  
#define POSITION_01_DCC_ADDRESS 200

// I decided to divide the turntable up into 10 Positions using #defines and mathc so it all scales with changes
// to the MS1,MS2,MS3 stepping jumpers above and to make the math tidy, but you assign positions how ever you like
#define POSITION_01 (HALF_TURN_STEPS / 10)

// This array contains the Turnout Positions which can have lines added/removed to suit your turntable 
TurnoutPosition turnoutPositions[] = {
  {POSITION_01_DCC_ADDRESS + 0, POSITION_01 * 1, POSITION_01 * 1 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 1, POSITION_01 * 2, POSITION_01 * 2 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 2, POSITION_01 * 3, POSITION_01 * 3 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 3, POSITION_01 * 4, POSITION_01 * 4 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 4, POSITION_01 * 5, POSITION_01 * 5 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 5, POSITION_01 * 6, POSITION_01 * 6 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 6, POSITION_01 * 7, POSITION_01 * 7 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 7, POSITION_01 * 8, POSITION_01 * 8 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 8, POSITION_01 * 9, POSITION_01 * 9 + HALF_TURN_STEPS },
  {POSITION_01_DCC_ADDRESS + 9, POSITION_01 *10, POSITION_01 *10 + HALF_TURN_STEPS },
};

// --------------------------------------------------------------------------------------------
// You shouldn't need to edit anything below this line unless you're needing to make big changes... ;)
// --------------------------------------------------------------------------------------------
#if defined(ALWAYS_MOVE_POSITIVE) && defined(ALWAYS_MOVE_NEGATIVE)
#error ONLY uncomment one of ALWAYS_MOVE_POSITIVE or ALWAYS_MOVE_NEGATIVE but NOT both
#endif

#define MAX_TURNOUT_POSITIONS (sizeof(turnoutPositions) / sizeof(TurnoutPosition))

// Setup the AccelStepper object for the A4988 Stepper Motor Driver
AccelStepper stepper1(AccelStepper::DRIVER, A4988_STEP_PIN, A4988_DIRECTION_PIN);

// Dcc Accessory Decoder object
NmraDcc  Dcc ;

// Variables to store the last DCC Turnout message Address and Direction  
uint16_t lastAddr = 0xFFFF ;
uint8_t lastDirection = 0xFF;
int     lastStep = 0;

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  Serial.print(F("notifyDccAccTurnoutOutput: "));
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;

  for (int i = 0; i < MAX_TURNOUT_POSITIONS ; i++)
  {
    if ((Addr == turnoutPositions[i].dccAddress) && ((Addr != lastAddr) || (Direction != lastDirection)) && OutputPower)
    {
      lastAddr = Addr ;
      lastDirection = Direction ;
      
      Serial.print(F("Moving to "));
      Serial.print(Direction ? F("Front") : F("Back"));
      Serial.print(F(" Position: "));
      Serial.print(i, DEC);
      Serial.print(F(" @ Step: "));

#ifdef A4988_ENABLE_PIN
      stepper1.enableOutputs();
#endif

      int newStep;
      if(Direction)
        newStep = turnoutPositions[i].positionFront;
      else
        newStep = turnoutPositions[i].positionBack;

      Serial.print(newStep, DEC);
      
      Serial.print(F("  Last Step: "));
      Serial.print(lastStep, DEC);
      
      int diffStep = newStep - lastStep;
      Serial.print(F("  Diff Step: "));
      Serial.print(diffStep, DEC);

#if defined ALWAYS_MOVE_POSITIVE
      Serial.print(F("  Positive"));       
      if(diffStep < 0)
        diffStep += FULL_TURN_STEPS;
        
#elif defined ALWAYS_MOVE_NEGATIVE
      Serial.print(F("  Negative"));       
      if(diffStep > 0)
        diffStep -= FULL_TURN_STEPS;
#else
      if(diffStep > HALF_TURN_STEPS)
        diffStep = diffStep - FULL_TURN_STEPS;
        
      else if(diffStep < -HALF_TURN_STEPS)
        diffStep = diffStep + FULL_TURN_STEPS;
#endif

      Serial.print(F("  Move: "));
      Serial.println(diffStep, DEC);
      stepper1.move(diffStep);

      lastStep = newStep;
      break;
    }
  }
};

#ifdef DISABLE_OUTPUTS_IDLE
bool lastIsRunningState ;
#endif 

void setupStepperDriver()
{
#ifdef A4988_ENABLE_PIN
  stepper1.setPinsInverted(false, false, true); // Its important that these commands are in this order
  stepper1.setEnablePin(A4988_ENABLE_PIN);    // otherwise the Outputs are NOT enabled initially
#endif
   
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);        // Sets the maximum permitted speed
  stepper1.setAcceleration(STEPPER_ACCELARATION); // Sets the acceleration/deceleration rate
  stepper1.setSpeed(STEPPER_SPEED);               // Sets the desired constant speed for use with runSpeed()

#ifdef A4988_ENABLE_PIN
  stepper1.enableOutputs();
#endif

#ifdef DISABLE_OUTPUTS_IDLE
  lastIsRunningState = stepper1.isRunning();
#endif
}

bool moveToHomePosition()
{
  Serial.println(F("Finding Home Sensor...."));

  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);

#ifdef ALWAYS_MOVE_NEGATIVE
  stepper1.move(0 - (FULL_TURN_STEPS * 2));
#else
  stepper1.move(FULL_TURN_STEPS * 2);
#endif  
  while(digitalRead(HOME_SENSOR_PIN) != HOME_SENSOR_ACTIVE_STATE)
    stepper1.run();

  if(digitalRead(HOME_SENSOR_PIN) == HOME_SENSOR_ACTIVE_STATE)
  {
    stepper1.stop();
    stepper1.setCurrentPosition(0);
    Serial.println(F("Found Home Position - Setting Current Position to 0"));
    return true;
  }
  else
    Serial.println(F("Home Position NOT FOUND - Check Sensor Hardware"));

  return false;  
}

void setupDCCDecoder()
{
  Serial.println(F("Setting up DCC Decorder..."));

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);   // Wait for the USB Device to Enumerate

  Serial.println(F("\nExample Stepper Motor Driver for DCC Turntable Control"));

  Serial.print(F("Full Rotation Steps: "));
  Serial.println(FULL_TURN_STEPS);

  Serial.print(F("Movement Strategy: "));
#if defined ALWAYS_MOVE_POSITIVE
  Serial.println(F("Positive Direction Only"));
#elif defined ALWAYS_MOVE_NEGATIVE
  Serial.println(F("Negative Direction Only"));
#else
  Serial.println(F("Shortest Distance"));
#endif

  for(uint8_t i = 0; i < MAX_TURNOUT_POSITIONS; i++)
  {
    Serial.print(F("DCC Addr: "));
    Serial.print(turnoutPositions[i].dccAddress);

    Serial.print(F(" Front: "));
    Serial.print(turnoutPositions[i].positionFront);

    Serial.print(F(" Back: "));
    Serial.println(turnoutPositions[i].positionBack);
  }
  
  setupStepperDriver();
  if(moveToHomePosition());
  { 
    setupDCCDecoder();

    // Fake a DCC Packet to cause the Turntable to move to Position 1
    notifyDccAccTurnoutOutput(POSITION_01_DCC_ADDRESS, 1, 1);
  }
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Process the Stepper Library
  stepper1.run();

#ifdef DISABLE_OUTPUTS_IDLE
  if(stepper1.isRunning() != lastIsRunningState)
  {
    lastIsRunningState = stepper1.isRunning();
    if(!lastIsRunningState)
    {
      stepper1.disableOutputs();
      Serial.println(F("Disable Stepper Outputs"));
    }
  }
#endif  
}
