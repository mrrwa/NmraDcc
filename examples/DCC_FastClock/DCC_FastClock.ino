#include <Arduino.h>
#include <NmraDcc.h>

//#include <compat/twi.h>  // I2C
//#include <U8g2lib.h>


#define SW_VERSION 1

NmraDcc  Dcc ;

// U8g2 constructor for display
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);  // 0.96 display

uint8_t fastClockHours = 0;
uint8_t fastClockMinutes = 0;
uint8_t fastClockWeekDay = 0;
uint8_t fastClockFactor = 0;
uint8_t fastClockUpdate = 0;
uint8_t  fastClockDayOfMonth = 0;
uint8_t  fastClockMonth = 0;
uint16_t fastClockYear = 0;
bool newClock = true;





//
// Is called when a RCN-211 Time packet is received
//
void notifyFastClockTime( uint8_t day, uint8_t hour, uint8_t minute, uint8_t factor, bool update)
{
      fastClockWeekDay = day;
      fastClockHours = hour;
      fastClockMinutes = minute;
      fastClockFactor = factor;
      fastClockUpdate = update;

      // Signal that a new value has arrived
      newClock = true;
}

void notifyFastClockDate( uint8_t dayOfMonth, uint8_t month, uint16_t year)
{
      fastClockDayOfMonth = dayOfMonth;
      fastClockMonth = month;
      fastClockYear = year;

      // Signal that a new value has arrived
      newClock = true;
}

void notifySystemTime( uint16_t millisSinceSystemStart)
{
  // Don't know what to do with this.
}




void setup() {
  // put your setup code here, to run once:


  // Setup which External Interrupt on Pin 2 and enable the Pull-Up (saves us from adding external 10k)
  Dcc.pin(0, 2, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.initAccessoryDecoder( MAN_ID_DIY, SW_VERSION, CV29_ACCESSORY_DECODER /* | CV29_OUTPUT_ADDRESS_MODE*/, 0 );

  // Startup U8g2 display
  //u8g2.begin();

}

void loop() {
 // MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( newClock) {

    char timemsg[12];
    char* daymsg = NULL;

    if ( fastClockFactor == 0) {
      sprintf( timemsg, "%s", "Paused");
    }
    else {
      sprintf( timemsg, "%2d:%02d", fastClockHours, fastClockMinutes);
      switch ( fastClockWeekDay) {
        case 0: daymsg = "Monday"; break;
        case 1: daymsg = "Tuesday"; break;
        case 2: daymsg = "Wednesday"; break;
        case 3: daymsg = "Thursday"; break;
        case 4: daymsg = "Friday"; break;
        case 5: daymsg = "Saturday"; break;
        case 6: daymsg = "Sunday"; break;
        default: daymsg = "?"; break;
      }
    }
      
    //u8g2.firstPage();
    //do {
    //    /* all graphics commands have to appear within the loop body. */    
    //
    //  u8g2.setFont(u8g2_font_t0_22_tr);
    //  if ( daymsg != NULL) {
    //    u8g2.drawStr( 10, 25, daymsg);
    //  }
    //  u8g2.drawStr( 10, 63, timemsg);
    //} while ( u8g2.nextPage() );

    newClock = false;
  }
}
