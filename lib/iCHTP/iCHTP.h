#ifndef ICHTP_H
#define ICHTP_H

#include <Arduino.h>
#include <Wire.h>

class iCHTP
{
	public:
		iCHTP();					// Initialize the iCHTP
		uint8_t Configure(uint8_t u8Address, uint8_t u8I2cChannel);
		uint8_t WriteReg(uint8_t u8Cmd, uint8_t u8Data0);
		uint8_t WriteRegTwoBytes(uint8_t u8Cmd, uint8_t u8Data0, uint8_t u8Data1);
		
		uint8_t ReadReg(uint8_t u8Cmd, uint8_t u8Len, uint8_t au8_RegsReturn[]);
	
	private:
		int _addr; 									// Address of sensor 
		uint8_t u8Channel;							//I2C-Kanal, an den das iC-Htp angeschlossen ist
		void openReg(uint8_t reg); 	    			// Points to a given register
		void  SwitchToI2cChan(uint8_t ucChannel);
};


#endif //#ifndef ICHTP_H
