
#define NUM_LIFT_LEVELS  7

#if defined(ARDUINO_AVR_UNO)

#define HOME_SENSOR_PIN     10

#define STEPPER_PULSE_PIN   11
#define STEPPER_ENABLE_PIN  12
#define STEPPER_DIR_PIN     13

#define STEPPER_MAX_SPEED             2100
#define STEPPER_NORMAL_ACCELERATION   5000
#define STEPPER_MAX_POSITION  20000U // Maximum Steps to allow the stepper to drive Up Saftey mechanism

#define BUTTON_OK    A3
#define BUTTON_DOWN  A2
#define BUTTON_UP    A1
#define BUTTON_HOME  A0

#elif defined (ARDUINO_AVR_NANO)
#endif

#define STEPPER_INC_SPEED             (STEPPER_MAX_SPEED / 10)
#define STEPPER_MAN_SPEED_CHANGE_MILLIS 5
#define STEPPER_EMERGENCY_STOP_ACCELERATION   100000
#define LIFT_LEVEL_NOT_SET -1

typedef struct PERSISTENT_VALUES
{
  uint8_t numLiftLevels;
  uint8_t lastLiftLevel;
  long    lastStepperPosition;
  long    listPositions[NUM_LIFT_LEVELS];
  uint16_t objectSignature;
} persistentValues;

#include <AccelStepper.h>
#include <EncButton2.h>
#include <elapsedMillis.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN, -1, -1, false);

EncButton2<EB_BTN> homeSensor(INPUT_PULLUP, HOME_SENSOR_PIN);

EncButton2<EB_BTN> btnOk(INPUT, BUTTON_OK);
EncButton2<EB_BTN> btnDown(INPUT, BUTTON_DOWN);
EncButton2<EB_BTN> btnUp(INPUT, BUTTON_UP);
EncButton2<EB_BTN> btnHome(INPUT, BUTTON_HOME);

void setup()
{
  Serial.begin(115200);
  Serial.println("\nStepper Motor Drive Test");  

  stepper.setEnablePin(STEPPER_ENABLE_PIN);
  stepper.setPinsInverted(true, false, true);
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);

  btnOk.setHoldTimeout(2000);
}

void moveStepper(long relative)
{
  stepper.enableOutputs();
  stepper.setAcceleration(STEPPER_NORMAL_ACCELERATION);
  stepper.move(relative); 
}

void stopStepper(void)
{
  stepper.setAcceleration(STEPPER_EMERGENCY_STOP_ACCELERATION);
  stepper.move(0); 
  stepper.stop();
  while(stepper.run());
  stepper.disableOutputs(); 
}

int lastSpeed = 0;
int newSpeed = 0;
bool wasRunning = false;
bool configMode = false;
elapsedMillis lastSpeedChange = 0;

void loop()
{
    //First check the Home Sensor and stop the motor if going in the down direction
  homeSensor.tick();
  if(homeSensor.state())
  {
    if(stepper.isRunning() && (lastSpeed <= 0))
    {
      stopStepper();

      Serial.print("Home Sensor Hit - LastSpeed: ");
      Serial.print(lastSpeed);
      Serial.print("  Last Position: ");
      Serial.println(stepper.currentPosition());

      newSpeed = 0;
      lastSpeed = newSpeed;
      stepper.setCurrentPosition(0);
    }
  }

    // Make sure we haven't gone beyond the end point of the traverser.
  if(stepper.currentPosition() >= STEPPER_MAX_POSITION)
  {
    if(stepper.isRunning() && (lastSpeed >= 0))
    {
      stopStepper();
      
      Serial.print("Maximum Position Hit - LastSpeed: ");
      Serial.print(lastSpeed);
      Serial.print("  Last Position: ");
      Serial.println(stepper.currentPosition());
      
      newSpeed = 0;
      lastSpeed = newSpeed;
    }
  }
  
  btnHome.tick();
  if(btnHome.click())
  {
    Serial.print("Home Click - Current Pos: "); Serial.println(stepper.currentPosition());
    if(stepper.isRunning())
    {
      newSpeed = 0;
      stopStepper();
    }
  }

  if(btnHome.held())
  {
    Serial.println("Home Held: Moving to Home Position");
    newSpeed = -STEPPER_MAX_SPEED;
  }
  
  btnOk.tick();
  if(btnOk.click())
  {
    Serial.print("Ok Click - Current Pos: "); Serial.println(stepper.currentPosition());
  }

  if(btnOk.held())
  {
    Serial.print("Ok Held - Current Pos: "); Serial.println(stepper.currentPosition());
  }

  btnDown.tick();
  if(btnDown.click() && (stepper.currentPosition() < STEPPER_MAX_POSITION) && (lastSpeed <= (STEPPER_MAX_SPEED - STEPPER_INC_SPEED)))
  {
    newSpeed = lastSpeed + STEPPER_INC_SPEED;
    lastSpeedChange = STEPPER_MAN_SPEED_CHANGE_MILLIS;
    Serial.print("Down Click - Current Pos: "); Serial.print(stepper.currentPosition()); Serial.print("  New Speed: "); Serial.println(newSpeed);
  }

  btnUp.tick();
  if(btnUp.click() && (homeSensor.state() == 0) && (lastSpeed >= -(STEPPER_MAX_SPEED - STEPPER_INC_SPEED)))
  {
    newSpeed = lastSpeed - STEPPER_INC_SPEED;
    lastSpeedChange = STEPPER_MAN_SPEED_CHANGE_MILLIS;
    Serial.print("Up Click - Current Pos: "); Serial.print(stepper.currentPosition()); Serial.print("  New Speed: "); Serial.println(newSpeed);
  }

  if(lastSpeed != newSpeed)
  {
//    Serial.print("Speed Change: Last: "); Serial.print(lastSpeed); Serial.print("  New: "); Serial.print(newSpeed);
//    Serial.print(" - Current Pos: "); Serial.print(stepper.currentPosition());
    
    if( newSpeed == 0)
    {
      lastSpeed = newSpeed;
      stopStepper();
      Serial.print("Speed Change: Stopped  Last: "); Serial.print(lastSpeed); Serial.print("  New: "); Serial.println(newSpeed);
    }

    else if(lastSpeedChange >= STEPPER_MAN_SPEED_CHANGE_MILLIS)
    {
      lastSpeedChange = 0;

      if(newSpeed > lastSpeed)
        lastSpeed++;
      else
        lastSpeed--;

      stepper.setSpeed(lastSpeed);
      stepper.enableOutputs();

//      Serial.print("  Set New Speed: "); Serial.println(newSpeed);
    }
  }
  
  if(lastSpeed)
    stepper.runSpeed();

  else
    stepper.run();

  if(!stepper.isRunning() && wasRunning)
  {
    Serial.println("Disable Outputs");
    stepper.disableOutputs();
  }
  wasRunning = stepper.isRunning();
}
