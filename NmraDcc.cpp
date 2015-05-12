//------------------------------------------------------------------------
//
// OpenDCC - NmraDcc.cpp 
//
// Copyright (c) 2011 Alex Shepherd
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      NmraDcc.cpp
// author:    Alex Shepherd
// webpage:   http://opendcc.org/
// history:   2011-06-26 Initial Version copied in from OpenDCC
//
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC Mobile and Stationary Decoders
//------------------------------------------------------------------------

#include "NmraDcc.h"
#include <avr/eeprom.h>

//------------------------------------------------------------------------
// DCC Receive Routine
//
// Howto:    uses two interrupts: a rising edge in DCC polarity triggers INTx
//           in INTx handler, Timer0 CompareB with a delay of 80us is started.
//           On Timer0 CompareB Match the level of DCC is evaluated and
//           parsed.
//
//                           |<-----116us----->|
//
//           DCC 1: _________XXXXXXXXX_________XXXXXXXXX_________
//                           ^-INTx
//                           |----87us--->|
//                                        ^Timer-INT: reads zero
//
//           DCC 0: _________XXXXXXXXXXXXXXXXXX__________________
//                           ^-INTx
//                           |----------->|
//                                        ^Timer-INT: reads one
//           
//------------------------------------------------------------------------

// The Timer0 prescaler is hard-coded in wiring.c 
#define TIMER_PRESCALER 64

// We will use a time period of 80us and not 87us as this gives us a bit more time to do other stuff 
#define DCC_BIT_SAMPLE_PERIOD (F_CPU * 70L / TIMER_PRESCALER / 1000000L)

#if (DCC_BIT_SAMPLE_PERIOD > 254)
#error DCC_BIT_SAMPLE_PERIOD too big, use either larger prescaler or slower processor
#endif
#if (DCC_BIT_SAMPLE_PERIOD < 8)
#error DCC_BIT_SAMPLE_PERIOD too small, use either smaller prescaler or faster processor
#endif

typedef enum
{
  WAIT_PREAMBLE = 0,
  WAIT_START_BIT,
  WAIT_DATA,
  WAIT_END_BIT
} 
DccRxWaitState ;

struct DccRx_t
{
  DccRxWaitState  State ;
  uint8_t         DataReady ;
  uint8_t         BitCount ;
  uint8_t         TempByte ;
  DCC_MSG         PacketBuf;
  DCC_MSG         PacketCopy;
} 
DccRx ;

typedef struct
{
  uint8_t   Flags ;
  uint8_t   OpsModeAddressBaseCV ;
  uint8_t   inServiceMode ;
  long      LastServiceModeMillis ;
  uint8_t   PageRegister ;  // Used for Paged Operations in Service Mode Programming
  uint8_t   DuplicateCount ;
  DCC_MSG   LastMsg ;
  uint8_t	ExtIntNum; 
  uint8_t	ExtIntPinNum;
#ifdef DCC_DEBUG
  uint8_t	IntCount;
  uint8_t	TickCount;
#endif
} 
DCC_PROCESSOR_STATE ;

DCC_PROCESSOR_STATE DccProcState ;

void ExternalInterruptHandler(void)
{
  OCR0B = TCNT0 + DCC_BIT_SAMPLE_PERIOD ;

#if defined(TIMSK0)
  TIMSK0 |= (1<<OCIE0B);  // Enable Timer0 Compare Match B Interrupt
  TIFR0  |= (1<<OCF0B);   // Clear  Timer0 Compare Match B Flag 
#elif defined(TIMSK)
  TIMSK |= (1<<OCIE0B);  // Enable Timer0 Compare Match B Interrupt
  TIFR  |= (1<<OCF0B);   // Clear  Timer0 Compare Match B Flag 
#endif
#ifdef DCC_DEBUG
	DccProcState.IntCount++;
#endif
}

ISR(TIMER0_COMPB_vect)
{
  uint8_t DccBitVal ;

  // Read the DCC input value, if it's low then its a 1 bit, otherwise it is a 0 bit
  DccBitVal = !digitalRead(DccProcState.ExtIntPinNum) ;

  // Disable Timer0 Compare Match B Interrupt
#if defined(TIMSK0)
  TIMSK0 &= ~(1<<OCIE0B);
#elif defined(TIMSK)
  TIMSK &= ~(1<<OCIE0B);
#endif

#ifdef DCC_DEBUG
	DccProcState.TickCount++;
#endif

  DccRx.BitCount++;

  switch( DccRx.State )
  {
  case WAIT_PREAMBLE:
    if( DccBitVal )
    {
      if( DccRx.BitCount > 10 )
        DccRx.State = WAIT_START_BIT ;
    }
    else
      DccRx.BitCount = 0 ;

    break;

  case WAIT_START_BIT:
    if( !DccBitVal )
    {
      DccRx.State = WAIT_DATA ;
      DccRx.PacketBuf.Size = 0;
      DccRx.PacketBuf.PreambleBits = 0;
      for(uint8_t i = 0; i< MAX_DCC_MESSAGE_LEN; i++ )
        DccRx.PacketBuf.Data[i] = 0;

      // We now have 1 too many PreambleBits so decrement before copying
      DccRx.PacketBuf.PreambleBits = DccRx.BitCount - 1 ;

      DccRx.BitCount = 0 ;
      DccRx.TempByte = 0 ;
    }
    break;

  case WAIT_DATA:
    DccRx.TempByte = ( DccRx.TempByte << 1 ) ;
    if( DccBitVal )
      DccRx.TempByte |= 1 ;

    if( DccRx.BitCount == 8 )
    {
      if( DccRx.PacketBuf.Size == MAX_DCC_MESSAGE_LEN ) // Packet is too long - abort
      {
        DccRx.State = WAIT_PREAMBLE ;
        DccRx.BitCount = 0 ;
      }
      else
      {
        DccRx.State = WAIT_END_BIT ;
        DccRx.PacketBuf.Data[ DccRx.PacketBuf.Size++ ] = DccRx.TempByte ;
      }
    }
    break;

  case WAIT_END_BIT:
    if( DccBitVal ) // End of packet?
    {
      DccRx.State = WAIT_PREAMBLE ;
      DccRx.PacketCopy = DccRx.PacketBuf ;
      DccRx.DataReady = 1 ;
    }
    else  // Get next Byte
    DccRx.State = WAIT_DATA ;

    DccRx.BitCount = 0 ;
    DccRx.TempByte = 0 ;
  }
}

void ackCV(void)
{
  if( notifyCVAck )
    notifyCVAck() ;
}

uint8_t validCV( uint16_t CV, uint8_t Writable )
{
  if( notifyCVResetFactoryDefault && (CV == CV_MANUFACTURER_ID )  && Writable )
	notifyCVResetFactoryDefault();
	
  if( notifyCVValid )
    return notifyCVValid( CV, Writable ) ;

  uint8_t Valid = 1 ;

  if( CV > E2END )
    Valid = 0 ;

  if( Writable && ( ( CV ==CV_VERSION_ID ) || (CV == CV_MANUFACTURER_ID ) ) )
    Valid = 0 ;

  return Valid ;
}

uint8_t readCV( uint16_t CV )
{
  uint8_t Value ;

  if( notifyCVRead )
    return notifyCVRead( CV ) ;

  Value = eeprom_read_byte( (uint8_t*) CV ) ;

  return Value ;
}

uint8_t writeCV( uint16_t CV, uint8_t Value)
{
  if( notifyCVWrite )
    return notifyCVWrite( CV, Value ) ;

  if( eeprom_read_byte( (uint8_t*) CV ) != Value )
  {
    eeprom_write_byte( (uint8_t*) CV, Value ) ;

    if( notifyCVChange )
      notifyCVChange( CV, Value) ;
  }

  return eeprom_read_byte( (uint8_t*) CV ) ;
}

uint16_t getMyAddr(void)
{
  uint16_t  Addr ;
  uint8_t   CV29Value ;

  CV29Value = readCV( CV_29_CONFIG ) ;

  if( CV29Value & 0b10000000 )  // Accessory Decoder? 
    Addr = ( readCV( CV_ACCESSORY_DECODER_ADDRESS_MSB ) << 6 ) | readCV( CV_ACCESSORY_DECODER_ADDRESS_LSB ) ;

  else   // Multi-Function Decoder?
  {
    if( CV29Value & 0b00100000 )  // Two Byte Address?
      Addr = ( readCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8 ) | readCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB ) ;

    else
      Addr = readCV( 1 ) ;
  }

  return Addr ;
}

void processDirectOpsOperation( uint8_t Cmd, uint16_t CVAddr, uint8_t Value )
{
  // is it a Byte Operation
  if( Cmd & 0x04 )
  {
    // Perform the Write Operation
    if( Cmd & 0x08 )
    {
      if( validCV( CVAddr, 1 ) )
      {
        if( writeCV( CVAddr, Value ) == Value )
          ackCV();
      }
    }

    else  // Perform the Verify Operation
    {  
      if( validCV( CVAddr, 0 ) )
      {
        if( readCV( CVAddr ) == Value )
          ackCV();
      }
    }
  }
  // Perform the Bit-Wise Operation
  else
  {
    uint8_t BitMask = (1 << (Value & 0x07) ) ;
    uint8_t BitValue = Value & 0x08 ;
    uint8_t BitWrite = Value & 0x10 ;

    uint8_t tempValue = readCV( CVAddr ) ;  // Read the Current CV Value

    // Perform the Bit Write Operation
    if( BitWrite )
    {
      if( validCV( CVAddr, 1 ) )
      {
        if( BitValue )
          tempValue |= BitMask ;     // Turn the Bit On

        else
          tempValue &= ~BitMask ;  // Turn the Bit Off

        if( writeCV( CVAddr, tempValue ) == tempValue )
          ackCV() ;
      }
    }

    // Perform the Bit Verify Operation
    else
    {
      if( validCV( CVAddr, 0 ) )
      {
        if( BitValue ) 
        {
          if( tempValue & BitMask )
            ackCV() ;
        }
        else
        {
          if( !( tempValue & BitMask)  )
            ackCV() ;
        }
      }
    }
  }
}

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
void processMultiFunctionMessage( uint16_t Addr, uint8_t Cmd, uint8_t Data1, uint8_t Data2 )
{
  uint8_t  speed ;
  uint16_t CVAddr ;

  uint8_t  CmdMasked = Cmd & 0b11100000 ;

  // If we are an Accessory Decoder
  if( DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER )
  {
    // and this isn't an Ops Mode Write or we are NOT faking the Multifunction Ops mode address in CV 33+34 or
    // it's not our fake address, then return
    if( ( CmdMasked != 0b11100000 ) || ( DccProcState.OpsModeAddressBaseCV == 0 ) )
      return ;

    uint16_t FakeOpsAddr = readCV( DccProcState.OpsModeAddressBaseCV ) | ( readCV( DccProcState.OpsModeAddressBaseCV + 1 ) << 8 ) ;
    uint16_t OpsAddr = Addr & 0x3FFF ;

    if( OpsAddr != FakeOpsAddr )
      return ;
  }

  // We are looking for FLAGS_MY_ADDRESS_ONLY but it does not match and it is not a Broadcast Address then return
  else if( ( DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY ) && ( Addr != getMyAddr() ) && ( Addr != 0 ) ) 
    return ;

  switch( CmdMasked )
  {
  case 0b00000000:  // Decoder Control
    switch( Cmd & 0b00001110 )
    {
    case 0b00000000:  
      if( notifyDccReset && ( Cmd & 0b00000001 ) ) // Hard Reset
        if( notifyDccReset)
          notifyDccReset( 1 ) ;
      break ;

    case 0b00000010:  // Factory Test
      break ;

    case 0b00000110:  // Set Decoder Flasg
      break ;

    case 0b00001010:  // Set Advanced Addressing
      break ;

    case 0b00001110:  // Decoder Acknowledgment
      break ;

    default:    // Reserved
      ;
    }
    break ;

  case 0b00100000:  // Advanced Operations
    switch( Cmd & 0b00011111 )
    {
    case 0b00011111:
      if( notifyDccSpeed )
      {
        switch( Data1 & 0b01111111 )
        {
        case 0b00000000:  
          speed = 1 ;
          break ;

        case 0b00000001:  
          speed = 0 ;
          break ;

        default:
          speed = (Data1 & 0b01111111) - 1 ;
        }

        notifyDccSpeed( Addr, speed, Data1 & 0b10000000, 127 ) ;
      }
    }
    break;

  case 0b01000000:
  case 0b01100000:
    if( notifyDccSpeed )
    {
      switch( Cmd & 0b00011111 )
      {
      case 0b00000000:  
      case 0b00010000:
        speed = 1 ;
        break ;

      case 0b00000001:  
      case 0b00010001:
        speed = 0 ;
        break ;

      default:
        // This speed is not quite right as 14 bit mode can happen and we should check CV29 but... 
        speed = (((Cmd & 0b00001111) << 1 ) | ((Cmd & 0b00010000) >> 4)) - 2 ;
      }

      notifyDccSpeed( Addr, speed, Cmd & 0b00100000, 28 ) ;
    }

    break;

  case 0b10000000:  // Function Group 0..4
    if( notifyDccFunc )
      notifyDccFunc( Addr, FN_0_4, Cmd & 0b00011111 ) ;
    break;

  case 0b10100000:  // Function Group 5..8
    if( notifyDccFunc)
    {
      if (Cmd & 0b00010000 )
        notifyDccFunc( Addr, FN_5_8,  Cmd & 0b00001111 ) ;
      else
        notifyDccFunc( Addr, FN_9_12, Cmd & 0b00001111 ) ;
    }
    break;

  case 0b11000000:  // Feature Expansion Instruction
  	switch(Cmd & 0b00011111)
  	{
  	case 0B00011110:
  	  if( notifyDccFunc )
	    notifyDccFunc( Addr, FN_13_20, Data1 ) ;
	  break;
	  
  	case 0B00011111:
  	  if( notifyDccFunc )
	    notifyDccFunc( Addr, FN_21_28, Data1 ) ;
	  break;
  	}
    break;

  case 0b11100000:  // CV Access
    CVAddr = ( ( ( Cmd & 0x03 ) << 8 ) | Data1 ) + 1 ;

    processDirectOpsOperation( Cmd, CVAddr, Data2 ) ;
    break;
  }
}
#endif

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
void processServiceModeOperation( DCC_MSG * pDccMsg )
{
  uint16_t CVAddr ;
  uint8_t Value ;

  if( pDccMsg->Size == 3) // 3 Byte Packets are for Address Only, Register and Paged Mode
  {
    uint8_t RegisterAddr ;

    RegisterAddr = pDccMsg->Data[0] & 0x07 ;
    Value = pDccMsg->Data[1] ;

    if( RegisterAddr == 5 )
    {
      DccProcState.PageRegister = Value ;
      ackCV();
    }

    else
    {
      if( RegisterAddr == 4 )
        CVAddr = CV_29_CONFIG ;

      else if( ( RegisterAddr <= 3 ) && ( DccProcState.PageRegister > 0 ) )
        CVAddr = ( ( DccProcState.PageRegister - 1 ) * 4 ) + RegisterAddr + 1 ;

      else
        CVAddr = RegisterAddr + 1 ;

      if( pDccMsg->Data[0] & 0x08 ) // Perform the Write Operation
      {
        if( validCV( CVAddr, 1 ) )
        {
          if( writeCV( CVAddr, Value ) == Value )
            ackCV();
        }
      }

      else  // Perform the Verify Operation
      {  
        if( validCV( CVAddr, 0 ) )
        {
          if( readCV( CVAddr ) == Value )
            ackCV();
        }
      }
    }
  }

  else if( pDccMsg->Size == 4) // 4 Byte Packets are for Direct Byte & Bit Mode
  {
    CVAddr = ( ( ( pDccMsg->Data[0] & 0x03 ) << 8 ) | pDccMsg->Data[1] ) + 1 ;
    Value = pDccMsg->Data[2] ;

    processDirectOpsOperation( pDccMsg->Data[0] & 0b00001100, CVAddr, Value ) ;
  }
}
#endif
void resetServiceModeTimer(uint8_t inServiceMode)
{
  // Set the Service Mode
  DccProcState.inServiceMode = inServiceMode ;
  
  DccProcState.LastServiceModeMillis = inServiceMode ? millis() : 0 ;
}

void clearDccProcState(uint8_t inServiceMode)
{
  resetServiceModeTimer( inServiceMode ) ;

  // Set the Page Register to it's default of 1 only on the first Reset
  DccProcState.PageRegister = 1 ;

  // Clear the LastMsg buffer and DuplicateCount in preparation for possible CV programming
  DccProcState.DuplicateCount = 0 ;
  memset( &DccProcState.LastMsg, 0, sizeof( DCC_MSG ) ) ;
}

void execDccProcessor( DCC_MSG * pDccMsg )
{
  if( ( pDccMsg->Data[0] == 0 ) && ( pDccMsg->Data[1] == 0 ) )
  {
    if( notifyDccReset )
      notifyDccReset( 0 ) ;

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    // If this is the first Reset then perform some one-shot actions as we maybe about to enter service mode
    if( DccProcState.inServiceMode )
      resetServiceModeTimer( 1 ) ;
    else
      clearDccProcState( 1 );
#endif
  }

  else
  {
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    if( DccProcState.inServiceMode && ( pDccMsg->Data[0] >= 112 ) && ( pDccMsg->Data[0] < 128 ) )
    {
      resetServiceModeTimer( 1 ) ;

      if( memcmp( pDccMsg, &DccProcState.LastMsg, sizeof( DCC_MSG ) ) )
      {
        DccProcState.DuplicateCount = 0 ;
        memcpy( &DccProcState.LastMsg, pDccMsg, sizeof( DCC_MSG ) ) ;
      }
      // Wait until you see 2 identicle packets before acting on a Service Mode Packet 
      else
      {
        DccProcState.DuplicateCount++ ;
        processServiceModeOperation( pDccMsg ) ;
      }
    }

    else
    {
      if( DccProcState.inServiceMode )
        clearDccProcState( 0 );	
#endif

      // Idle Packet
      if( ( pDccMsg->Data[0] == 0b11111111 ) && ( pDccMsg->Data[1] == 0 ) )
      {
        if( notifyDccIdle )
          notifyDccIdle() ;
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (7-bit address)
      else if( pDccMsg->Data[0] < 128 )
        processMultiFunctionMessage( pDccMsg->Data[0], pDccMsg->Data[1], pDccMsg->Data[2], pDccMsg->Data[3] ) ;  

      // Basic Accessory Decoders (9-bit) & Extended Accessory Decoders (11-bit)
      else if( pDccMsg->Data[0] < 192 )
#else
      else if( ( pDccMsg->Data[0] >= 128 ) && ( pDccMsg->Data[0] < 192 ) )
#endif
      {
        if( DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER )
        {
          uint16_t BoardAddress ;
          uint8_t  OutputAddress ;
          uint8_t  OutputIndex ;
          uint16_t Address ;

          BoardAddress = ( ( (~pDccMsg->Data[1]) & 0b01110000 ) << 2 ) | ( pDccMsg->Data[0] & 0b00111111 ) ;

          // If we're filtering was it my board address Our or a broadcast address
          if( ( DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY ) && ( BoardAddress != getMyAddr() ) && ( BoardAddress != 511 ) )
            return;

          OutputAddress = pDccMsg->Data[1] & 0b00000111 ;
          
          OutputIndex = OutputAddress >> 1;

          Address = ( ( ( BoardAddress - 1 ) << 2 ) | OutputIndex ) + 1 ;

          if(pDccMsg->Data[1] & 0b10000000)
          {
            if( notifyDccAccState )
              notifyDccAccState( Address, BoardAddress, OutputAddress, pDccMsg->Data[1] & 0b00001000 ) ;
          }

          else
          {
            if( notifyDccSigState )
              notifyDccSigState( Address, OutputIndex, pDccMsg->Data[2] ) ;
          }
        }
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (14-bit address)
      else if( pDccMsg->Data[0] < 232 )
      {
        uint16_t Address ;
        Address = ( pDccMsg->Data[0] << 8 ) | pDccMsg->Data[1];
        processMultiFunctionMessage( Address, pDccMsg->Data[2], pDccMsg->Data[3], pDccMsg->Data[4] ) ;  
      }
#endif
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    }
#endif
  }
}

void initDccProcessor( uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV )
{
}

NmraDcc::NmraDcc()
{
}

void NmraDcc::pin( uint8_t ExtIntNum, uint8_t ExtIntPinNum, uint8_t EnablePullup)
{
  DccProcState.ExtIntNum = ExtIntNum;
  DccProcState.ExtIntPinNum = ExtIntPinNum;
	
  pinMode( ExtIntPinNum, INPUT );
  if( EnablePullup )
    digitalWrite(ExtIntPinNum, HIGH);
}

void NmraDcc::init( uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV )
{
  // Clear all the static member variables
  memset( &DccRx, 0, sizeof( DccRx) );

#ifdef TCCR0A
  // Change Timer0 Waveform Generation Mode from Fast PWM back to Normal Mode
  TCCR0A &= ~((1<<WGM01)|(1<<WGM00));
#else  
#error NmraDcc Library requires a processor with Timer0 Output Compare B feature 
#endif
  attachInterrupt( DccProcState.ExtIntNum, ExternalInterruptHandler, RISING);

  DccProcState.Flags = Flags ;
  DccProcState.OpsModeAddressBaseCV = OpsModeAddressBaseCV ;

  uint8_t cv29Mask = Flags & 0b11000000 ; // peal off the top two bits

  writeCV( 7, VersionId ) ;
  writeCV( 8, ManufacturerId ) ;

  // Set the Bits that control Multifunction or Accessory behaviour
  // and if the Accessory decoder optionally handles Output Addressing 
  writeCV( CV_29_CONFIG, ( readCV( CV_29_CONFIG ) & ~cv29Mask ) | cv29Mask ) ;

  clearDccProcState( 0 );
}

uint8_t NmraDcc::getCV( uint16_t CV )
{
  return readCV(CV);
}

uint8_t NmraDcc::setCV( uint16_t CV, uint8_t Value)
{
  return writeCV(CV,Value);
}

uint8_t NmraDcc::isSetCVReady(void)
{
  if(notifyIsSetCVReady)
	return notifyIsSetCVReady();
	
  return eeprom_is_ready();
}

#ifdef DCC_DEBUG
uint8_t NmraDcc::getIntCount(void)
{
  return DccProcState.IntCount;
}

uint8_t NmraDcc::getTickCount(void)
{
  return DccProcState.TickCount;
}

uint8_t NmraDcc::getState(void)
{
  return DccRx.State;
}

uint8_t NmraDcc::getBitCount(void)
{
  return DccRx.BitCount;
}
#endif

uint8_t NmraDcc::process()
{
  if( DccProcState.inServiceMode )
  {
    if( (millis() - DccProcState.LastServiceModeMillis ) > 20L )
    {
      clearDccProcState( 0 ) ;
    }
  }

  if( DccRx.DataReady )
  {
    // We need to do this check with interrupts disabled
    cli();
    Msg = DccRx.PacketCopy ;
    DccRx.DataReady = 0 ;
    sei();
    
    uint8_t xorValue = 0 ;
    
    for(uint8_t i = 0; i < DccRx.PacketCopy.Size; i++)
      xorValue ^= DccRx.PacketCopy.Data[i];

    if(xorValue)
      return 0 ;
    else
		{
			if( notifyDccMsg )
				notifyDccMsg( &Msg );
		
      execDccProcessor( &Msg );
    }
    return 1 ;
  }

  return 0 ;  
};