# NmraDcc
NMRA Digital Command Control (DCC) Library

**Warning** as of version 1.4.4 support has been removed for the following two call-back functions, which will cause your sketch to silently stop working:

	extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State )
	extern void notifyDccSigState( uint16_t Addr, uint8_t OutputIndex, uint8_t State) 

