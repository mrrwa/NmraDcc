#include <AccelStepper.h>   // Requires AccelStepper Library  - http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <EncButton2.h>     // Requires EncButton library     - https://github.com/GyverLibs/EncButton
#include <elapsedMillis.h>  // Requires elapsedMillis library - https://github.com/pfeerick/elapsedMillis

#define OPTIMIZE_I2C 1

#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <EEPROM.h>
#include <NmraDcc.h>

// You can print every DCC packet by un-commenting the line below
//#define NOTIFY_DCC_MSG

// Define the Arduino Pin to connect to the DCC input signal
#define DCC_PIN 2

// Define the DCC Turnout Address to select the first level = 1 
#define DCC_ACCESSORY_DECODER_BASE_ADDRESS  500

// Define the manimus numbr of Levels
#define NUM_LIFT_LEVELS  8

#define PROGRAM_NAME     "Fahrstuhl"
#define PROGRAM_VERSION  "1.1"

// Locate the Persistant State storage EEPROM space well above the DCC Accessory Decoder CV Storage
#define EEPROM_BASE_ADDR  100
#define EEPROM_VALID_DATA_SIGNATURE 0xA5A5
// Uncomment the line below to force the EEPROM values to be reset to defaults
//#define EEPROM_FORCE_RELOAD_DEFAULT_VALUES

#define BUTTON_LONG_PRESS_DELAY 2000

// Uncomment ONE of the next 2 lines to enable AJS or UWE Board Settings
// #define AJS_BOARD_SETTINGS
#define UWE_BOARD_SETTINGS

#if defined(AJS_BOARD_SETTINGS)  // Setting for AJS Dev System

// Uncomment the next line to reverse the direction of the stepper movement
#define REVERSE_STEPPER_DIRECTION

#define HOME_SENSOR_PIN     10

#define STEPPER_PULSE_PIN   11
#define STEPPER_ENABLE_PIN  12
#define STEPPER_DIR_PIN     13

#define STEPPER_MAX_SPEED             2100
#define STEPPER_NORMAL_ACCELERATION   5000
#define STEPPER_MAX_POSITION  300000U // Maximum Steps to allow the stepper to drive Up Saftey mechanism

#define BUTTON_MANUAL     A3
#define BUTTON_DOWN       A2
#define BUTTON_UP         A1
#define BUTTON_STOP_HOME  A0

long defaultPositions[NUM_LIFT_LEVELS] = {1000, 4000, 7000, 10000, 13000, 16000, 19000, 22000}; // Default positions

#define STEPPER_INC_SPEED             (STEPPER_MAX_SPEED / 10)

#define OLED_DISPLAY_I2C_ADDRESS  0x3C

#elif defined (UWE_BOARD_SETTINGS)  // Setting for Uwe's Fahrstuhl System

// Uncomment the next line to reverse the direction of the stepper movement
//#define REVERSE_STEPPER_DIRECTION

#define HOME_SENSOR_PIN     7

#define STEPPER_PULSE_PIN   4
#define STEPPER_ENABLE_PIN  5
#define STEPPER_DIR_PIN     6

#define STEPPER_MAX_SPEED             2100
#define STEPPER_NORMAL_ACCELERATION   5000
#define STEPPER_MAX_POSITION  1970000U // Maximum Steps to allow the stepper to drive Up Saftey mechanism

#define BUTTON_MANUAL     8
#define BUTTON_DOWN       9
#define BUTTON_UP         10
#define BUTTON_STOP_HOME  11

long defaultPositions[NUM_LIFT_LEVELS] = {0, 161064, 32500, 483284, 645326, 808041, 1967457, 1130774}; // Default positions

#define STEPPER_INC_SPEED             (STEPPER_MAX_SPEED / 2)

#define OLED_DISPLAY_I2C_ADDRESS  0x3C

#else
#error No Board Settings Defined
#endif

SSD1306AsciiWire oled;

#define STEPPER_MAN_SPEED_CHANGE_MILLIS 5
#define STEPPER_EMERGENCY_STOP_ACCELERATION   100000
#define LIFT_LEVEL_NOT_SET -1

typedef struct
{
  uint8_t numLiftLevels;
  uint8_t lastLiftLevel;
  long    lastStepperPosition;
  long    levelPositions[NUM_LIFT_LEVELS];
  uint16_t objectSignature;
} PERSISTENT_VALUES;

PERSISTENT_VALUES persistentValues;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_DIR_PIN, -1, -1, false);

EncButton2<EB_BTN> homeSensor(INPUT_PULLUP, HOME_SENSOR_PIN);

EncButton2<EB_BTN> btnManual(INPUT, BUTTON_MANUAL);
EncButton2<EB_BTN> btnDown(INPUT, BUTTON_DOWN);
EncButton2<EB_BTN> btnUp(INPUT, BUTTON_UP);
EncButton2<EB_BTN> btnStopHome(INPUT, BUTTON_STOP_HOME);

// NMRA DCC Accessory Decoder object
NmraDcc  Dcc;

void displayLevel(int newLevel)
{
  oled.setCursor(0,0);
  oled.set2X();
  oled.print("Level: ");
  oled.print(newLevel);
  oled.clearToEOL();
}

void displayMessage(const char* Msg)
{
  oled.setCursor(0,4);
  oled.set2X();
  oled.print(Msg); oled.clearToEOL();
}

void displayMessageNumber(const char* Msg, int Number)
{
  oled.setCursor(0,4);
  oled.set2X();
  oled.print(Msg);
  oled.print(Number);
  oled.clearToEOL();
}

void displayPosition(long newPosition)
{
  oled.setCursor(0,7);
  oled.set1X();
  oled.print("Pos: ");
  oled.print(newPosition);
  oled.clearToEOL();
}

void initPersistentValues()
{
  EEPROM.get(EEPROM_BASE_ADDR, persistentValues);

#ifdef EEPROM_FORCE_RELOAD_DEFAULT_VALUES
  persistentValues.objectSignature = 0;
#endif

  if(persistentValues.objectSignature != EEPROM_VALID_DATA_SIGNATURE)
  {
    Serial.println("initPersistentValues: set detault values");

    persistentValues.numLiftLevels = NUM_LIFT_LEVELS;
    persistentValues.lastLiftLevel = 0;
    persistentValues.lastStepperPosition = 0;
    persistentValues.objectSignature = EEPROM_VALID_DATA_SIGNATURE;
    for(uint8_t i = 0; i < NUM_LIFT_LEVELS; i++)
      persistentValues.levelPositions[i] = defaultPositions[i];

    EEPROM.put(EEPROM_BASE_ADDR, persistentValues);
  }
  else
    Serial.println("initPersistentValues: restored values from EEPROM");
}

void setup()
{
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);

  Serial.println(); Serial.print(PROGRAM_NAME); Serial.print("  Version: "); Serial.println(PROGRAM_VERSION);

  initPersistentValues();

  Wire.begin();
  Wire.setClock(400000L);

  oled.setFont(cp437font8x8);

  oled.begin(&Adafruit128x64, OLED_DISPLAY_I2C_ADDRESS);
  oled.clear();
  oled.println(PROGRAM_NAME);
  oled.println();
  oled.print("Ver: "); oled.println(PROGRAM_VERSION);
  oled.println();
  oled.print("Max Levels:  "); oled.println(NUM_LIFT_LEVELS);
  oled.println();
  oled.print("Used Levels: "); oled.println(persistentValues.numLiftLevels);
  delay(2000);
  oled.clear();
  displayLevel(persistentValues.lastLiftLevel + 1);
  displayPosition(persistentValues.lastStepperPosition);

  stepper.setCurrentPosition(persistentValues.lastStepperPosition);

  stepper.setEnablePin(STEPPER_ENABLE_PIN);
#ifdef REVERSE_STEPPER_DIRECTION
  stepper.setPinsInverted(true, false, true);
#else
  stepper.setPinsInverted(false, false, true);
#endif

  stepper.setMaxSpeed(STEPPER_MAX_SPEED);

  btnStopHome.setHoldTimeout(BUTTON_LONG_PRESS_DELAY);
  btnManual.setHoldTimeout(BUTTON_LONG_PRESS_DELAY);
  
    // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(DCC_PIN, 1);
  
    // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);
}

void stepperMoveTo(long newPosition)
{
  stepper.enableOutputs();
  stepper.setAcceleration(STEPPER_NORMAL_ACCELERATION);
  stepper.moveTo(newPosition);
}

void stepperMove(long newRelPosition)
{
  stepper.enableOutputs();
  stepper.setAcceleration(STEPPER_NORMAL_ACCELERATION);
  stepper.move(newRelPosition);
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
bool homing = false;
elapsedMillis lastSpeedChange = 0;


  // This function is called whenever a normal DCC Turnout Packet is received
  // The DCC Turnout Address is checked to see if it is within the range used to Select Elevator levels and starts a Move if a new level is selected 
void notifyDccAccTurnoutOutput(uint16_t receivedAddress, uint8_t direction, uint8_t outputPower)
{
  if((receivedAddress >= DCC_ACCESSORY_DECODER_BASE_ADDRESS) && (receivedAddress < (DCC_ACCESSORY_DECODER_BASE_ADDRESS + (NUM_LIFT_LEVELS/2))))
  {
    uint8_t newLevel = (receivedAddress - DCC_ACCESSORY_DECODER_BASE_ADDRESS) * 2 + direction;
    if(persistentValues.lastLiftLevel != newLevel)
    {
      persistentValues.lastLiftLevel = newLevel;

      long newPos = persistentValues.levelPositions[persistentValues.lastLiftLevel];
      stepperMoveTo(newPos);

      Serial.print("notifyDccAccTurnoutOutput: Move to Level: "); Serial.print(persistentValues.lastLiftLevel); Serial.print("  Pos: "); Serial.println(newPos);

      displayMessageNumber("Mv To: ", persistentValues.lastLiftLevel + 1);
    }
  }
}

void loop()
{
  Dcc.process();
  
    //First check the Home Sensor and stop the motor if going in the down direction
  homeSensor.tick();
  if(homeSensor.state())
  {
    if((configMode || homing) && stepper.isRunning() && (lastSpeed <= 0))
    {
      stopStepper();

      Serial.print("Home Sensor Hit - LastSpeed: ");
      Serial.print(lastSpeed);
      Serial.print("  Last Position: ");
      Serial.println(stepper.currentPosition());

      newSpeed = 0;
      lastSpeed = newSpeed;

      persistentValues.lastLiftLevel = 0;
      persistentValues.lastStepperPosition = 0;
      stepper.setCurrentPosition(persistentValues.lastStepperPosition);

      EEPROM.put(EEPROM_BASE_ADDR, persistentValues);

      if(homing)
      {
        long newPos = persistentValues.levelPositions[persistentValues.lastLiftLevel];
        stepperMoveTo(newPos);
        Serial.print("Home Sensor Hit: Move To: "); Serial.print(persistentValues.lastLiftLevel); Serial.print("  Pos: "); Serial.println(newPos);
        homing = false;
      }
    }
  }

    // Make sure we haven't gone beyond the end point of the traverser.
  if(stepper.currentPosition() >= STEPPER_MAX_POSITION)
  {
    if(configMode && stepper.isRunning() && (lastSpeed >= 0))
    {
      stopStepper();

      Serial.print("Maximum Position Hit - LastSpeed: ");
      Serial.print(lastSpeed);
      Serial.print("  Last Position: ");
      Serial.println(stepper.currentPosition());

      newSpeed = 0;
      lastSpeed = newSpeed;

      displayMessage("At Max");
    }
  }

  btnStopHome.tick();
  if(btnStopHome.press())
  {
    Serial.print("StopHome Click - Current Pos: "); Serial.println(stepper.currentPosition());
    displayMessage("Stop");
    if(stepper.isRunning())
    {
      newSpeed = 0;
      stopStepper();
    }
  }

  if(btnStopHome.held())
  {
    Serial.println("StopHome Held: Moving to Home Position");
    displayMessage("Homing");
    homing = true;
    newSpeed = -STEPPER_MAX_SPEED;
  }

  btnManual.tick();
  if(btnManual.press())
  {
    Serial.print("Manual Press - Current Pos: "); Serial.println(stepper.currentPosition());
    if(configMode)
    {
      configMode = false;
      Serial.println("Home Click - Exit Manual Mode");
    }
  }

  if(btnManual.held())
  {
    Serial.print("Manual Held - Enter Manual Mode Pos: "); Serial.println(stepper.currentPosition());
    configMode = true;
  }

  btnDown.tick();
  if(configMode)
  {
    if((btnDown.press() || btnDown.step()) && (stepper.currentPosition() < STEPPER_MAX_POSITION) && (lastSpeed <= (STEPPER_MAX_SPEED - STEPPER_INC_SPEED)))
    {
      newSpeed = lastSpeed + STEPPER_INC_SPEED;
      lastSpeedChange = STEPPER_MAN_SPEED_CHANGE_MILLIS;
      Serial.print("Down Press - Current Pos: "); Serial.print(stepper.currentPosition()); Serial.print("  New Speed: "); Serial.println(newSpeed);

      displayMessage("Down");
    }
  }

  else if((btnDown.press() || btnDown.step()) && persistentValues.lastLiftLevel > 0)
  {
    Serial.print("Down Press - Current Level: "); Serial.print(persistentValues.lastLiftLevel);
    persistentValues.lastLiftLevel--;
    long newPos = persistentValues.levelPositions[persistentValues.lastLiftLevel];
    stepperMoveTo(newPos);
    Serial.print("  Move To: "); Serial.print(persistentValues.lastLiftLevel); Serial.print("  Pos: "); Serial.println(newPos);

    displayMessageNumber("Dn To: ", persistentValues.lastLiftLevel + 1);
  }

  btnUp.tick();
  if(configMode)
  {
    if((btnUp.press() || btnDown.step()) && (homeSensor.state() == 0) && (lastSpeed >= -(STEPPER_MAX_SPEED - STEPPER_INC_SPEED)))
    {
      newSpeed = lastSpeed - STEPPER_INC_SPEED;
      lastSpeedChange = STEPPER_MAN_SPEED_CHANGE_MILLIS;
      Serial.print("Up Press - Current Pos: "); Serial.print(stepper.currentPosition()); Serial.print("  New Speed: "); Serial.println(newSpeed);

      displayMessage("Up");
    }
  }

  else if((btnUp.press() || btnDown.step()) && (persistentValues.lastLiftLevel < (persistentValues.numLiftLevels - 1)))
  {
    Serial.print("Up Press - Current Level: "); Serial.print(persistentValues.lastLiftLevel);
    persistentValues.lastLiftLevel++;
    long newPos = persistentValues.levelPositions[persistentValues.lastLiftLevel];
    stepperMoveTo(newPos);
    Serial.print("  Move To: "); Serial.print(persistentValues.lastLiftLevel); Serial.print("  Pos: "); Serial.println(newPos);

    displayMessageNumber("Up To: ", persistentValues.lastLiftLevel + 1);
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

    displayLevel(persistentValues.lastLiftLevel + 1);
    displayMessage("");
    persistentValues.lastStepperPosition = stepper.currentPosition();
    displayPosition(persistentValues.lastStepperPosition);
    EEPROM.put(EEPROM_BASE_ADDR, persistentValues);
  }
  wasRunning = stepper.isRunning();
}

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
