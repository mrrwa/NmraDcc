# NmraDcc
NMRA Digital Command Control (DCC) Library

This library allows you to interface to a NMRA DCC track signal and receive DCC commands.

The library currently supports the AVR ATTiny84/85 & ATMega88/168/328/32u4 and Teensy 3.x using the INT0/1 Hardware Interrupt and micros() ONLY and no longer uses Timer0 Compare Match B, which makes it much more portable to other platforms.

**Warning** as of version 1.4.4 support has been removed for the following two call-back functions, which will cause your sketch to silently stop working:

	extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State )
	extern void notifyDccSigState( uint16_t Addr, uint8_t OutputIndex, uint8_t State) 

