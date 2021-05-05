#ifndef DAC_LTC2635_H
#define DAC_LTC2635_H

#include <Arduino.h>
#include <Wire.h>

#define TEMP_SET 0
#define DAC_B 1
#define DAC_C 2
#define DAC_D 3

class LTC2635
{
	public:
		LTC2635(uint8_t addr, uint8_t u8I2cChannel);					// Initialize the LTC2635
		void Configure();
        void SetDac(unsigned char ucDacAddress , uint16_t ui16DacValue);

		uint8_t WriteReg(uint8_t u8Cmd, uint8_t u8Data0, uint8_t u8Data1);
		uint8_t ReadReg(uint8_t u8Cmd, uint8_t u8Len);
	
	private:
		int _addr; 									// Address of sensor 
		uint8_t u8Channel;							//I2C-Kanal, an den das iC-Htp angeschlossen ist
		void openReg(uint8_t reg); 	    			// Points to a given register
        void SwitchToI2cChan(uint8_t ucChannel);
};


#endif //#ifndef DAC_LTC2635_H
