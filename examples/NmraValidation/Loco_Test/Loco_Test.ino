/***********************************************************************************************
 *
 * This sketch tests the NmraDcc library as a multifunction decoder.
 * Author: Kenneth West
 *         kgw4449@gmail.com
 *
 * This sketch has added the printf() function to the Print class.
 * You can find instructions for doing this here:
 *    http://playground.arduino.cc/Main/Printf
 *
 * This sketch is based on NmraDcc library NmraDccMultiFunctionDecoder_1 example.
 * You can find the library here:
 *    https://github.com/mrrwa/NmraDcc
 *
 * This Example shows how to use the library with the Iowa Scaled Engineering ARD-DCCSHIELD
 * You can find out more about this DCC Interface here:
 *    http://www.iascaled.com/store/ARD-DCCSHIELD
 *
 * For more information refer to the file: README.md here:
 *    https://github.com/IowaScaledEngineering/ard-dccshield
 *
 * This demo assumes the following Jumper settings on the ARD-DCCSHIELD
 *
 * JP1 - I2C Pull-Up Resistors                        - Don't Care
 * JP2 - (Pins 1-2) I2C /IORST JP2                    - Don't-Care
 * JP2 - (Pins 3-4) - DCC Signal to Arduino Pin       - OFF
 * JP3 - I2C /INT and /OE                             - Don't-Care
 * JP4 - DCC Signal to Arduino Pin                    - D2 ON
 * JP5 - Arduino Powered from DCC                     - User Choice
 * JP6 - Boards without VIO                           - User Choice
 * JP7 - Enable Programming ACK                       - 1-2 ON 3-4 ON 
 * 
 * The connections are as follows:
 * 
 * Pin  Name          Mode          Description
 * ----------------------------------------------------------------------------------
 * D2   DCC_PIN       INPUT_PULLUP  DCC input signal.
 * A1   ACK_PIN       OUTPUT        CV acknowledge control.
 * A0   MOTOR_A_PIN   OUTPUT        Motor output A.
 * D13  MOTOR_B_PIN   OUTPUT        Motor output B.
 * D12  FUNC_A_PIN    OUTPUT        Function output A.
 * D11  FUNC_B_PIN    OUTPUT        Function output B.
 * D4   SCOPE_PIN     OUTPUT        SCOPE trigger.
 * 
 **********************************************************************************************/
// Column locations.
//3456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456
//       1         2         3         4         5         6         7         8         9     9
//const unsigned long FAKE_LONG     =          10000UL; // Fake variable for column positions.

#include <NmraDcc.h>

// Uncomment this line to print out minimal status information.
#define DCC_STATUS

// Uncomment this line to issue scope trigger at beginning of motor callback.
#define DO_SCOPE

const byte            VER_MAJOR     =                2; // Major version in CV 7
const byte            VER_MINOR     =                1; // Minor version in CV 112    
const byte            DCC_PIN       =                2; // DCC input pin.
const int             ACK_PIN       =               A1; // CV acknowledge pin.
const byte            MOTOR_A_PIN   =               A0; // Motor out A pin.
const byte            MOTOR_B_PIN   =               13; // Motor out B pin.
const byte            FUNC_A_PIN    =               12; // Function A pin.
const byte            FUNC_B_PIN    =               11; // Function B pin.
const byte            SCOPE_PIN     =                4; // Scope trigger pin.
const byte            CV_VERSION    =                7; // Decoder version.
const byte            CV_MANUF      =                8; // Manufacturer ID.
const byte            CV_MANUF_01   =              112; // Manufacturer Unique 01.
const byte            MANUF_ID      =       MAN_ID_DIY; // Manufacturer ID in CV 8.
const byte            DECODER_ADDR  =                3; // Decoder address.
const byte            SP_ESTOP      =                0; // Emergency stop speed.
const byte            SP_STOP       =                1; // Stop speed value.
const unsigned long   DELAY_TIME    =               50; // Delay time in ms.

// Note: Set FORCE_CV_WRITE true to force CVs to update if just the address is changed.
const bool            FORCE_CV_WRITE  =          false; // Set true to force CV write.

enum ShowTypes {
  S_IDLE                            =             0x01, // Show Idle packets.
  S_RESET                           =             0x02, // Show Reset packets.
  S_DCC                             =             0x04, // Show DCC information.
  S_ACK                             =             0x08, // Basic acknowledge off.
  S_ALL                             =             0x80, // Show raw packet data.
};

bool                  MotorFwd      =            false; // Motor direction.
bool                  FuncOn        =             true; // Function on/off.
byte                  ShowData      =             0x00; // Packet information to show.
unsigned long         EndTime       =                0; // End time in ms.

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDR},         // Short address.

  // Reload these just in case they are writeen by accident.
  {CV_VERSION, VER_MAJOR},                                  // Decoder version.
  {CV_MANUF,   MANUF_ID },                                  // Manufacturer ID.

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},               // Extended address MSB.
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, DECODER_ADDR},    // Extended address LSB.

  {CV_29_CONFIG,                         CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
  {CV_MANUF_01,                          VER_MINOR},        // Minor decoder version.
};

NmraDcc  Dcc ;

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  Serial.println(F("notifyCVResetFactoryDefault called."));
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// Uncomment the #define below to print all Speed Packets
#define NOTIFY_DCC_SPEED
#ifdef  NOTIFY_DCC_SPEED
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
#ifdef DO_SCOPE
  digitalWrite(SCOPE_PIN, HIGH);
  digitalWrite(SCOPE_PIN, LOW);
#endif // DO_SCOPE
  
  if (ShowData & S_DCC) {
    Serial.print("notifyDccSpeed: Addr: ");
    Serial.print(Addr,DEC);
    Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
    Serial.print(" Speed: ");
    Serial.print(Speed,DEC);
    Serial.print(" Steps: ");
    Serial.print(SpeedSteps,DEC);
    Serial.print(" Dir: ");
    Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  }
  
  if ((Speed > SP_STOP) && (Dir == DCC_DIR_REV)) {
    setMotor(false);
  }
  else {
    setMotor(true);
  }
};
#endif // NOTIFY_DCC_SPEED

// Uncomment the #define below to print all Function Packets
#define NOTIFY_DCC_FUNC
#ifdef  NOTIFY_DCC_FUNC
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  if (ShowData & S_DCC) {
    Serial.print("notifyDccFunc: Addr: ");
    Serial.print(Addr,DEC);
    Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
    Serial.print("  Function Group: ");
    Serial.print(FuncGrp,DEC);
  }

  switch( FuncGrp ) {
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE    
     case FN_0:
       if (ShowData & S_DCC) {
         Serial.print(" FN0: ");
         Serial.println((FuncState & FN_BIT_00) ? "1  " : "0  ");
       }
       break;
#endif // NMRA_DCC_ENABLE_14_SPEED_STEP_MODE
       
     case FN_0_4:       
       if (ShowData & S_DCC) {
         Serial.print(" FN 0: ");
         Serial.print((FuncState & FN_BIT_00) ? "1  ": "0  ");
         Serial.print(" FN 1-4: ");
         Serial.print((FuncState & FN_BIT_01) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_02) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_03) ? "1  ": "0  ");
         Serial.println((FuncState & FN_BIT_04) ? "1  ": "0  ");
       }
       
       if (FuncState & FN_BIT_00) {
         setFunc(true);
       }
       else {
         setFunc(false);
       }
       break;
    
     case FN_5_8:
       if (ShowData & S_DCC) {
         Serial.print(" FN 5-8: ");
         Serial.print((FuncState & FN_BIT_05) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_06) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_07) ? "1  ": "0  ");
         Serial.println((FuncState & FN_BIT_08) ? "1  ": "0  ");
       }
       break;
    
     case FN_9_12:
       if (ShowData & S_DCC) {
         Serial.print(" FN 9-12: ");
         Serial.print((FuncState & FN_BIT_09) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_10) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_11) ? "1  ": "0  ");
         Serial.println((FuncState & FN_BIT_12) ? "1  ": "0  ");
       }
       break;

     case FN_13_20:
       if (ShowData & S_DCC) {
         Serial.print(" FN 13-20: ");
         Serial.print((FuncState & FN_BIT_13) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_14) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_15) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_16) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_17) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_18) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_19) ? "1  ": "0  ");
         Serial.println((FuncState & FN_BIT_20) ? "1  ": "0  ");
       }
       break;
  
     case FN_21_28:
       if (ShowData & S_DCC) {
         Serial.print(" FN 21-28: ");
         Serial.print((FuncState & FN_BIT_21) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_22) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_23) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_24) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_25) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_26) ? "1  ": "0  ");
         Serial.print((FuncState & FN_BIT_27) ? "1  ": "0  ");
         Serial.println((FuncState & FN_BIT_28) ? "1  ": "0  ");
       }
       break;  
   }
}
#endif // NOTIFY_DCC_FUNC

// Uncomment the #define below to print all Reset Packets.
#define NOTIFY_DCC_RESET
#ifdef  NOTIFY_DCC_RESET
void notifyDccReset(uint8_t hardReset) {
  setMotor(true);
  setFunc(false);
  if (ShowData & S_RESET) {
    Serial.printf(F("notifyDccReset: %6s.\n"), hardReset ? "HARD" : "NORMAL");
  }
}
#endif // NOTIFY_DCC_RESET

// This function is called whenever a DCC Idle packet is received.
// Uncomment to print Idle Packets
#define NOTIFY_DCC_IDLE
#ifdef  NOTIFY_DCC_IDLE
void notifyDccIdle()
{
  if (ShowData & S_IDLE) { // Show Idle packets if S_IDLE is set. 
    Serial.println("notifyDccIdle: Idle received") ;
  }
}
#endif // NOTIFY_DCC_IDLE

// Uncomment the #define below to print changed CV values.
#define NOTIFY_CV_CHANGE
#ifdef  NOTIFY_CV_CHANGE
void notifyCVChange( uint16_t CV, uint8_t Value) {
  Serial.printf(F("notifyCVChange: CV %4u value changed to %3u 0x%02X.\n"), CV, Value, Value);
}
#endif // NOTIFY_CV_CHANGE

// This function is called when any DCC packet is received.
// Uncomment to print all DCC Packets
#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  if (ShowData & S_ALL) {  // Show all packets if S_ALL is set.
    Serial.print("notifyDccMsg: ") ;
    for(uint8_t i = 0; i < Msg->Size; i++)
    {
      Serial.print(Msg->Data[i], HEX);
      Serial.write(' ');
    }
    Serial.println();
  }
}
#endif // NOTIFY_DCC_MSG

// This function is called by the NmraDcc library when a DCC ACK needs to be sent.
// Calling this function should cause an increased 60ma current drain on
// the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
#ifdef DO_SCOPE
  digitalWrite(SCOPE_PIN, HIGH);
  digitalWrite(SCOPE_PIN, LOW);
#endif // DO_SCOPE
  
  if ((ShowData & S_ACK) == 0x00) { // Send CV acknowledge current pulse.                                           [
    Serial.println("notifyCVAck: Current pulse sent") ;
  
    digitalWrite( ACK_PIN, HIGH );
    delay( 6 );  
    digitalWrite( ACK_PIN, LOW );
  }
  else {                          // Suppress CV acknowledge current pulse.    
    Serial.println("notifyCVAck: Current pulse NOT sent") ;
  }
}

void setMotor(bool fwd) {
  #ifdef DCC_STATUS
  if (MotorFwd != fwd) {
    Serial.printf(F("Motor    changed to %3s.\n"), fwd ? "FWD" : "REV");
  }
  #endif // DCC_STATUS
  MotorFwd = fwd;
  digitalWrite(MOTOR_A_PIN,  fwd);
  digitalWrite(MOTOR_B_PIN, !fwd);
}

void setFunc(bool on) {
  #ifdef DCC_STATUS
  if (FuncOn != on) {
    Serial.printf(F("Function changed to %3s.\n"), on ? "ON" : "OFF");
  }
  #endif // DCC_STATUS
  FuncOn = on;
  digitalWrite(FUNC_A_PIN,  !on);
  digitalWrite(FUNC_B_PIN,   on);
}

void setup()
{
  Serial.begin(115200);
  Serial.print(F("NMRA Dcc Loco_Test "));
  Serial.printf(F("    Version %d.%d, Build date %s %s\n"), VER_MAJOR,
                                                            VER_MINOR,
                                                            __DATE__,
                                                            __TIME__);
  Serial.println(F("Cmds: a - All, d - DCC, i - Idle, r - Reset, c - CV Ack off,"));
  Serial.println(F("      <Other> - Everything off."));
  
  // Set MotorFwd false and FuncOn true to force the set the output.
  MotorFwd = false;
  FuncOn   = true;
  
  // Configure motor and fuction output pin pairs.
  pinMode(     MOTOR_A_PIN, OUTPUT);
  pinMode(     MOTOR_B_PIN, OUTPUT);
  setMotor(true);
  pinMode(     FUNC_A_PIN,  OUTPUT);
  pinMode(     FUNC_B_PIN,  OUTPUT);
  setFunc(false);

  // Configure Scope trigger output.
#ifdef DO_SCOPE
  pinMode(     SCOPE_PIN,   OUTPUT);
  digitalWrite(SCOPE_PIN,   LOW);
#endif // DO_SCOPE
  
  // Configure the DCC CV Programing ACK and set it LOW to keep the ACK current off.
  pinMode(     ACK_PIN,     OUTPUT );
  digitalWrite(ACK_PIN,     LOW );
  
  // Setup which External Interrupt, the Pin it's associated with that we're using
  // and enable the Pull-Up.
  Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, 1);
  
  // Reset the CVs to factory default if the manuf. ID or major version do not match.
  // Do this before init() since it sets these CVs.
  if (  ( FORCE_CV_WRITE)                                                 ||
        ((Dcc.getCV(CV_29_CONFIG) & CV29_ACCESSORY_DECODER) != 0)         ||
        ( Dcc.getCV(CV_MANUFACTURER_ID)                     != MANUF_ID)  ||
        ( Dcc.getCV(CV_VERSION_ID)                          != VER_MAJOR) ||
        ( Dcc.getCV(CV_MANUF_01)                            != VER_MINOR))
{
    notifyCVResetFactoryDefault();
  }
  
  Dcc.init( MANUF_ID, VER_MAJOR, FLAGS_MY_ADDRESS_ONLY, 0 );
  
  // Make sure CV_MANUF_01 CV matches VER_MINOR.
  Dcc.setCV(CV_MANUF_01, VER_MINOR);
  
  Serial.println(F("Init Done"));

  // Flush serial prior to entering loop().
  Serial.flush();
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if (Serial.available()) {
    // Get the new byte and process it.
    switch ((char)Serial.read()) {
    case 'a':
      if (ShowData & S_ALL) {
        ShowData &= ~S_ALL;
      }
      else {
        ShowData |= S_ALL;
      }
      Serial.println(ShowData & S_ALL ? "All   ON" : "All   OFF");
      break;
    case 'd':
      if (ShowData & S_DCC) {
        ShowData &= ~S_DCC;
      }
      else {
        ShowData |= S_DCC;
      }
      Serial.println(ShowData & S_DCC ? "DCC   ON" : "DCC   OFF");
      break;
    case 'i':
      if (ShowData & S_IDLE) {
        ShowData &= ~S_IDLE;
      }
      else {
        ShowData |= S_IDLE;
      }
      Serial.println(ShowData & S_IDLE ? "Idle  ON" : "Idle  OFF");
      break;
    case 'r':
      if (ShowData & S_RESET) {
        ShowData &= ~S_RESET;
      }
      else {
        ShowData |= S_RESET;
      }
      Serial.println(ShowData & S_RESET ? "Reset ON" : "Reset OFF");
      break;
    case 'c':
      if (ShowData & S_ACK) {
        ShowData &= ~S_ACK;
      }
      else {
        ShowData |= S_ACK;
      }
      Serial.println(ShowData & S_ACK ? "Ack   OFF" : "Ack   ON");
      break;
    case '\n':
    case '\r':
      break;
    default:
      if (ShowData != 0x00) {
        EndTime = millis() + DELAY_TIME;
      }
      break;
    }
  }

  if ((EndTime != 0) && (millis() > EndTime)) {
    Serial.printf(F("Clearing ShowData 0x%02x\n"), ShowData);
    ShowData = 0x00;
    EndTime  = 0;
  }
  
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    Serial.printf(F("CV %4u reset to factory default %3u 0x%02X.\n"),
                FactoryDefaultCVs[FactoryDefaultCVIndex].CV,
                FactoryDefaultCVs[FactoryDefaultCVIndex].Value,
                FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
    Dcc.setCV(  FactoryDefaultCVs[FactoryDefaultCVIndex].CV,
                FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}

