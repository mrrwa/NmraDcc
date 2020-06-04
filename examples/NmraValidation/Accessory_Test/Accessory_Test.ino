/***********************************************************************************************
 *
 * This sketch test the NmraDcc library as an accessory decoder.
 * Author: Kenneth West
 *         kgw4449@gmail.com
 *
 * This sketch has added the printf() function to the Print class.
 * You can find instructions for doing this here:
 *    http://playground.arduino.cc/Main/Printf
 *
 * It is based on NmraDcc library NmraDccAccessoryDecoder_1 example.
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
 * A0   ACC_A_PIN     OUTPUT        Accessory output A.
 * D13  ACC_B_PIN     OUTPUT        Accessory output B.
 * D4   SCOPE_PIN     OUTPUT        SCOPE trigger pin.
 * 
 ***********************************************************************************************
  */

#include <NmraDcc.h>

// Uncomment this line to print out minimal status information.
#define DCC_STATUS

// Uncomment this line to issue scope trigger at beginning of motor callback.
#define DO_SCOPE

// Uncomment this line to handle Basic Accessory decoders.
#define ACCESSORY_DCC

// Uncomment this line to handle Signal decoders.
#define SIGNAL_DCC

const byte            VER_MAJOR     =                2; // Major version in CV 7
const byte            VER_MINOR     =                1; // Minor version in CV 112    
const byte            DCC_PIN       =                2; // DCC input pin.
const int             ACK_PIN       =               A1; // CV acknowledge pin.
const byte            SCOPE_PIN     =                4; // Scope trigger pin.
const byte            ACC_A_PIN     =               A0; // Motor out A pin.
const byte            ACC_B_PIN     =               13; // Motor out B pin.
const byte            CV_VERSION    =                7; // Decoder version.
const byte            CV_MANUF      =                8; // Manufacturer ID.
const byte            CV_MANUF_01   =              112; // Manufacturer Unique 01.
const byte            MANUF_ID      =       MAN_ID_DIY; // Manufacturer ID in CV 8.
const unsigned long   DELAY_TIME    =               50; // Delay time in ms.

// The following constant is the 9 bit accessory address. It is followed by the 6 LSB bits that
// go into CV1 and the 6 MBB bits that go into CV9.
// Note: Set FORCE_CVS true to force CVs to update if just the address is changed.
const bool            FORCE_CVS     =            false; // Set true to force CV write.
const uint16_t        ACC_ADDR      =                1; // 11 bit address:
                                                        // 0 - broadcast, 1 to 2044 - specific.
const uint16_t        BD_ADDR       = ((ACC_ADDR - 1) >> 2) + 1;
                                                        // 9 bit board address:
                                                        // 0 - broadcast, 1 to  511 - specific.
const byte            BD_LSB        =   BD_ADDR & 0x3f; // Board address LSB.
const byte            BD_MSB        =     BD_ADDR >> 6; // Board address MSB.

// CV29_DEFAULT is the factory hardware default for CV29.
const byte            CV29_DEFAULT  = CV29_OUTPUT_ADDRESS_MODE |
                                      CV29_ACCESSORY_DECODER;

enum ACC_DIR {
  REV                               =                0, // Direction is reverse.
  NORM                              =                1, // Direction is normal.
};

enum ShowTypes {
  S_IDLE                            =             0x01, // Show Idle packets.
  S_RESET                           =             0x02, // Show Reset packets.
  S_DCC                             =             0x04, // Show DCC information.
  S_ACK                             =             0x08, // Basic acknowledge off.
  S_ALL                             =             0x80, // Show raw packet data.
};

ACC_DIR               AccDir        =              REV; // Accessory direction.
byte                  ShowData      =             0x00; // Packet information to show.
unsigned long         EndTime       =                0; // End time in ms.

NmraDcc  Dcc ;

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB,     BD_LSB},     // Accessory address LSB.
  {CV_ACCESSORY_DECODER_ADDRESS_MSB,     BD_MSB},     // Accessory address MSB.
  
  // Reload these just in case they are writeen by accident.
  {CV_VERSION, VER_MAJOR},                                  // Decoder version.
  {CV_MANUF,   MANUF_ID },                                  // Manufacturer ID.
  
  {CV_29_CONFIG,                         CV29_DEFAULT},     // Configuration CV.
  {CV_MANUF_01,                          VER_MINOR},        // Minor decoder version.
};

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  Serial.println(F("notifyCVResetFactoryDefault called."));
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This function is called whenever a normal DCC Turnout Packet is received
#define NOTITY_DCC_ACC_TURNOUT_BOARD
#ifdef  NOTITY_DCC_ACC_TURNOUT_BOARD
void notifyDccAccTurnoutBoard( uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower )
{
  if (ShowData & S_DCC) {
    Serial.print("notifyDccAccTurnoutBoard: ") ;
    Serial.print(BoardAddr,DEC) ;
    Serial.print(',');
    Serial.print(OutputPair,DEC) ;
    Serial.print(',');
    Serial.print(Direction,DEC) ;
    Serial.print(',');
    Serial.println(OutputPower, HEX) ;
  }
}
#endif // NOTITY_DCC_ACC_TURNOUT_BOARD

// This function is called whenever a normal DCC Turnout Packet is received
// if NOTIFY_DCC_ACC_TURNOUT and ACCESSORY_DCC are defined.
#define NOTITY_DCC_ACC_TURNOUT_OUTPUT
#if  defined(NOTITY_DCC_ACC_TURNOUT_OUTPUT) && defined(ACCESSORY_DCC)
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
#ifdef DO_SCOPE
  digitalWrite(SCOPE_PIN, HIGH);
  digitalWrite(SCOPE_PIN, LOW);
#endif // DO_SCOPE
  
  if (ShowData & S_DCC) {
    Serial.print("notifyDccAccTurnoutOutput: ") ;
    Serial.print(Addr,DEC) ;
    Serial.print(',');
    Serial.print(Direction,DEC) ;
    Serial.print(',');
    Serial.println(OutputPower, HEX) ;
  }
  
  // Set the output to the given direction for just the ACC_ADDR output
  //   since this callback is called for all 4 output addresses.
  if (Addr == ACC_ADDR) {
    setAcc(Direction ? REV : NORM);
  }
}
#endif // NOTITY_DCC_ACC_TURNOUT_OUTPUT

// This function is called whenever a DCC Signal Aspect Packet is received
// if NOTIFY_DCC_SIG_STATE and SIGNAL_DCC are defined.
#define NOTITY_DCC_SIG_STATE
#if  defined(NOTITY_DCC_SIG_STATE) && defined(SIGNAL_DCC)
void notifyDccSigOutputState( uint16_t Addr, uint8_t State)
{
#ifdef DO_SCOPE
  digitalWrite(SCOPE_PIN, HIGH);
  digitalWrite(SCOPE_PIN, LOW);
#endif // DO_SCOPE
  
  if (ShowData & S_DCC) {
    Serial.print("notifyDccSigOutputState: ") ;
    Serial.print(Addr,DEC) ;
    Serial.print(',');
    Serial.println(State, DEC) ;
  }

  // Set the output to the given direction for 1st ACC_ADDR output.
  if (Addr == ACC_ADDR) {
    setAcc(State == 0 ? REV : NORM);
  }
}
#endif // NOTITY_DCC_SIG_STATE

// This function is called whenever a DCC Reset packet is received.
// Uncomment to print Reset Packets
#define NOTIFY_DCC_RESET
#ifdef  NOTIFY_DCC_RESET
void notifyDccReset(uint8_t hardReset )
{
  if (ShowData & S_RESET) {  // Show Reset packets if S_RESET is set.  
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

void setAcc(ACC_DIR dir) {
bool                  on;                               // Output on/off.
  on = dir == NORM ? true : false;
  #ifdef DCC_STATUS
  if (AccDir != dir) {
    Serial.printf(F("Accessory changed to %4s.\n"), on ? "NORM" : "REV");
  }
  #endif // DCC_STATUS
  AccDir = dir;
  digitalWrite(ACC_A_PIN,   on);
  digitalWrite(ACC_B_PIN,  !on);
}

void setup()
{
  Serial.begin(115200);
  Serial.printf(F("Starting Accessory_Test Version %d.%d, Build date %s %s\n"),
                                                            VER_MAJOR,
                                                            VER_MINOR,
                                                            __DATE__,
                                                            __TIME__);
  Serial.printf(F("Default ACC_ADDR %4u "), ACC_ADDR);
  Serial.printf(F("BD_ADDR %4u 0x%04X, BD_LSB %3u 0x%02X, BD_MSB %3u 0x%02X.\n"),
                                                            BD_ADDR,
                                                            BD_ADDR,
                                                            BD_LSB,
                                                            BD_LSB,
                                                            BD_MSB,
                                                            BD_MSB);
  Serial.println(F("Cmds: a - All, d - DCC, i - Idle, r - Reset, c - CV Ack off,"));
  Serial.println(F("      <Other> - Everything off."));

  // Set AccOn REV to force setAcc() to set the  output.
  AccDir = REV;
  
  // Configure motor and fuction output pin pairs.
  pinMode(     ACC_A_PIN,   OUTPUT);
  pinMode(     ACC_B_PIN,   OUTPUT);
  setAcc(NORM);
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode(      ACK_PIN,    OUTPUT );
  digitalWrite( ACK_PIN,    LOW);
  
  // Configure Scope trigger output.
#ifdef DO_SCOPE
  pinMode(     SCOPE_PIN,   OUTPUT);
  digitalWrite(SCOPE_PIN,   LOW);
#endif // DO_SCOPE

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, 1);
  
  // Reset the CVs to factory default if the decode type,manuf. ID or major version
  // do not match. Do this before init() since it sets these CVs.
  if (  (FORCE_CVS)                                  ||
        (Dcc.getCV(CV_29_CONFIG)        != CV29_DEFAULT)  ||
        (Dcc.getCV(CV_MANUFACTURER_ID)  != MANUF_ID)      ||
        (Dcc.getCV(CV_VERSION_ID)       != VER_MAJOR)     ||
        (Dcc.getCV(CV_MANUF_01)         != VER_MINOR))
  {
    notifyCVResetFactoryDefault();
  }

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MANUF_ID, VER_MAJOR,  // FLAGS_MY_ADDRESS_ONLY       |
                                  CV29_DEFAULT, 0 );

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
    Serial.printf(F("Clearing ShowData 0x%02X\n"), ShowData);
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

