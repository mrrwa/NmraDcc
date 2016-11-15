//------------------------------------------------------------------------
//
// Model Railroading with Arduino - NmraDcc.h 
//
// Copyright (c) 2008 - 2105 Alex Shepherd
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      NmraDcc.h
// author:    Alex Shepherd
// webpage:   http://mrrwa.org/
// history:   2008-03-20 Initial Version
//            2011-06-26 Migrated into Arduino library from OpenDCC codebase
//            2014 Added getAddr to NmraDcc  Geoff Bunza
//            2015-11-06 Martin Pischky (martin@pischky.de):
//                       Experimental Version to support 14 speed steps
//                       and new signature of notifyDccSpeed and notifyDccFunc
//
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC MutliFunction and Stationary Decoders
//
//------------------------------------------------------------------------

// Uncomment the following Line to Enable Service Mode CV Programming
#define NMRA_DCC_PROCESS_SERVICEMODE

// Uncomment the following line to Enable MutliFunction Decoder Operations
#define NMRA_DCC_PROCESS_MULTIFUNCTION

// Uncomment the following line to Enable 14 Speed Step Support
//#define NMRA_DCC_ENABLE_14_SPEED_STEP_MODE

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef NMRADCC_IS_IN
#define NMRADCC_IS_IN

#define MAX_DCC_MESSAGE_LEN 6    // including XOR-Byte

typedef struct
{
	uint8_t	Size ;
	uint8_t	PreambleBits ;
	uint8_t Data[MAX_DCC_MESSAGE_LEN] ;
} DCC_MSG ;

//--------------------------------------------------------------------------
//  This section contains the NMRA Assigned DCC Manufacturer Id Codes that
//  are used in projects
//
//  This value is to be used for CV8 
//--------------------------------------------------------------------------

#define MAN_ID_JMRI             0x12
#define MAN_ID_DIY              0x0D
#define MAN_ID_SILICON_RAILWAY  0x21

//--------------------------------------------------------------------------
//  This section contains the Product/Version Id Codes for projects
//
//  This value is to be used for CV7 
//
//  NOTE: Each Product/Version Id Code needs to be UNIQUE for that particular
//  the DCC Manufacturer Id Code
//--------------------------------------------------------------------------

// Product/Version Id Codes allocated under: MAN_ID_JMRI
 
// Product/Version Id Codes allocated under: MAN_ID_DIY

// Standard CV Addresses
#define CV_ACCESSORY_DECODER_ADDRESS_LSB       1
#define CV_ACCESSORY_DECODER_ADDRESS_MSB       9

#define CV_MULTIFUNCTION_PRIMARY_ADDRESS       1
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB 17
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB 18

#define CV_VERSION_ID                          7
#define CV_MANUFACTURER_ID                     8
#define CV_29_CONFIG                          29

#if defined(ESP8266)
#include <spi_flash.h>
#define MAXCV     SPI_FLASH_SEC_SIZE
#else
#define MAXCV                                 E2END     // the upper limit of the CV value currently defined to max memory.
#endif

typedef enum {
    CV29_LOCO_DIR            = 0b00000001,	/** bit 0: Locomotive Direction: "0" = normal, "1" = reversed */
    CV29_F0_LOCATION         = 0b00000010,	/** bit 1: F0 location: "0" = bit 4 in Speed and Direction instructions, "1" = bit 4 in function group one instruction */
    CV29_APS								 = 0b00000100,	/** bit 2: Alternate Power Source (APS) "0" = NMRA Digital only, "1" = Alternate power source set by CV12 */
    CV29_ADV_ACK             = 0b00001000, 	/** bit 3: ACK, Advanced Acknowledge mode enabled if 1, disabled if 0 */
    CV29_SPEED_TABLE_ENABLE  = 0b00010000, 	/** bit 4: STE, Speed Table Enable, "0" = values in CVs 2, 4 and 6, "1" = Custom table selected by CV 25 */
    CV29_EXT_ADDRESSING      = 0b00100000,	/** bit 5: "0" = one byte addressing, "1" = two byte addressing */
    CV29_OUTPUT_ADDRESS_MODE = 0b01000000,	/** bit 6: "0" = Decoder Address Mode "1" = Output Address Mode */
    CV29_ACCESSORY_DECODER   = 0b10000000,	/** bit 7: "0" = Multi-Function Decoder Mode "1" = Accessory Decoder Mode */
} CV_29_BITS;

typedef enum {
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE
    SPEED_STEP_14 = 15,			/**< ESTOP=0, 1 to 15 */
#endif
    SPEED_STEP_28 = 29,			/**< ESTOP=0, 1 to 29 */
    SPEED_STEP_128 = 127		/**< ESTOP=0, 1 to 127 */ 
} DCC_SPEED_STEPS;

typedef enum {
    DCC_DIR_REV = 0,     /** The locomotive to go in the reverse direction */
    DCC_DIR_FWD = 1,     /** The locomotive should move in the forward direction */
} DCC_DIRECTION;

typedef enum {
    DCC_ADDR_SHORT,      /** Short address is used. The range is 0 to 127. */
    DCC_ADDR_LONG,       /** Long Address is used. The range is 1 to 10239 */
} DCC_ADDR_TYPE;

typedef enum
{
	FN_0_4 = 1,
	FN_5_8,
	FN_9_12,
	FN_13_20,
	FN_21_28,
#ifdef NMRA_DCC_ENABLE_14_SPEED_STEP_MODE
  FN_0				 /** function light is controlled by base line package (14 speed steps) */
#endif  
} FN_GROUP;

#define FN_BIT_00	0x10
#define FN_BIT_01	0x01
#define FN_BIT_02	0x02
#define FN_BIT_03	0x04
#define FN_BIT_04	0x08

#define FN_BIT_05	0x01
#define FN_BIT_06	0x02
#define FN_BIT_07	0x04
#define FN_BIT_08	0x08

#define FN_BIT_09	0x01
#define FN_BIT_10	0x02
#define FN_BIT_11	0x04
#define FN_BIT_12	0x08

#define FN_BIT_13	0x01
#define FN_BIT_14	0x02
#define FN_BIT_15	0x04
#define FN_BIT_16	0x08
#define FN_BIT_17	0x10
#define FN_BIT_18	0x20
#define FN_BIT_19	0x40
#define FN_BIT_20	0x80

#define FN_BIT_21	0x01
#define FN_BIT_22	0x02
#define FN_BIT_23	0x04
#define FN_BIT_24	0x08
#define FN_BIT_25	0x10
#define FN_BIT_26	0x20
#define FN_BIT_27	0x40
#define FN_BIT_28	0x80

//#define DCC_DBGVAR
#ifdef DCC_DBGVAR
typedef struct countOf_t {
    unsigned long Tel;
    unsigned long Err;
}countOf_t ;

extern struct countOf_t countOf;
#endif

class NmraDcc
{
  private:
    DCC_MSG Msg ;
    
  public:
    NmraDcc();

// Flag values to be logically ORed together and passed into the init() method
#define FLAGS_MY_ADDRESS_ONLY				0x01	// Only process DCC Packets with My Address
#define FLAGS_OUTPUT_ADDRESS_MODE		0x40  // CV 29/541 bit 6
#define FLAGS_DCC_ACCESSORY_DECODER	0x80  // CV 29/541 bit 7
	void pin( uint8_t ExtIntNum, uint8_t ExtIntPinNum, uint8_t EnablePullup); 
  void init( uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV );
  void initAccessoryDecoder( uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV );
  uint8_t process();
  uint8_t getCV( uint16_t CV );
  uint8_t setCV( uint16_t CV, uint8_t Value);
	uint8_t isSetCVReady( void );
	uint16_t getAddr(void);
	
// #define DCC_DEBUG
#ifdef DCC_DEBUG
	uint8_t getIntCount(void);
	uint8_t getTickCount(void);
	uint8_t getBitCount(void);
	uint8_t getState(void);
  uint8_t getNestedIrqCount(void);
#endif

};

/************************************************************************************
    Call-back functions
************************************************************************************/

#if defined (__cplusplus)
	extern "C" {
#endif

extern void notifyDccReset(uint8_t hardReset ) __attribute__ ((weak));
extern void notifyDccIdle(void) __attribute__ ((weak));

extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps ) __attribute__ ((weak));
extern void notifyDccSpeedRaw( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Raw) __attribute__ ((weak));

extern void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) __attribute__ ((weak));

extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State ) __attribute__ ((weak));
extern void notifyDccAccTurnoutBoard( uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower ) __attribute__ ((weak));
extern void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) __attribute__ ((weak));

extern void notifyDccSigState( uint16_t Addr, uint8_t OutputIndex, uint8_t State) __attribute__ ((weak));

extern void    notifyDccMsg( DCC_MSG * Msg ) __attribute__ ((weak));

extern uint8_t notifyCVValid( uint16_t CV, uint8_t Writable ) __attribute__ ((weak));
extern uint8_t notifyCVRead( uint16_t CV) __attribute__ ((weak));
extern uint8_t notifyCVWrite( uint16_t CV, uint8_t Value) __attribute__ ((weak));
extern uint8_t notifyIsSetCVReady(void) __attribute__ ((weak));
extern void    notifyCVChange( uint16_t CV, uint8_t Value) __attribute__ ((weak));
extern void    notifyCVResetFactoryDefault(void) __attribute__ ((weak));

extern void    notifyCVAck(void) __attribute__ ((weak));

#if defined (__cplusplus)
}
#endif

#endif
