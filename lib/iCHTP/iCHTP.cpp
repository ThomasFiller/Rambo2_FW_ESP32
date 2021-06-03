#include "iCHTP.h"
#include <Wire.h>
#include "globalconsts.h"

iCHTP::iCHTP()
{
  
}

void  iCHTP::SwitchToI2cChan(uint8_t ucChannel) //Schaltet auf einen der 3 I2C-Kan�le durch Multiplexen des SCL-Signals
{
  if(ValBit(ucChannel, 0)) digitalWrite(GPIO_SELECT_I2C_2, HIGH); 	else digitalWrite(GPIO_SELECT_I2C_2, LOW);
  if(ValBit(ucChannel, 1)) digitalWrite(GPIO_SELECT_I2C, HIGH); 	else digitalWrite(GPIO_SELECT_I2C, LOW);
}

uint8_t iCHTP::Configure(uint8_t u8Address, uint8_t u8I2cChannel)
{
	_addr = u8Address;
	u8Channel = u8I2cChannel;
	Wire.begin(); // Join I2C bus

    SwitchToI2cChan(u8I2cChannel);
//DEBUG("fertig: SwitchToI2cChan(" + (String)u8I2cChannel + ")" );

    //1. iC-HTP starts in operation mode with default configuration. INITRAM, PDOVBLx and PDOVDD error bits must be set in STATUSx, DISC1 (addr. 0x10, bit 3) and DISC2 (addr. 0x15, bit 3) are set to 1.
    if (WriteReg(0x1C, 0x02, 0x00)) return 1;//Timout, weil der angesprochene iC-HTP nicht reagiert;
    //2. Write MODE(1:0) = "10" register (addr. 0x1C) to enable the configuration mode.
                                                              //In Configuration mode, the configuration memory (addr. 0x10 to 0x1F) can be written and read back to check a correct communication 
                                                              //without changing the present configured operation state of the iC-HTP(S.40)
    //3. Configure the laser channels.
WriteReg(0x10, 0xFF, 0x00);
//WriteReg(u8Address, 0x10, 0xF7, 0x00);
    WriteReg(0x11, 0xFF, 0x00);  
    WriteReg(0x13, REFREG_OFFSET, 0x00);   
    WriteReg(0x14, 0x00, 0x00);  
WriteReg(0x15, 0xFF, 0x00);
//WriteReg(u8Address, 0x15, 0xDF, 0x00);
    WriteReg(0x16, 0xFF, 0x00);  
    WriteReg(0x18, REFREG_OFFSET, 0x00);  
    WriteReg(0x19, 0x00, 0x00);  

    WriteReg(0x1A, 0x66, 0x00);  
    WriteReg(0x1B, 0x3F, 0x00);  
    WriteReg(0x1E, REG_0x1E_OFFSET + 0x33, 0x00);  
    //4. Read back to verify a correct data transfer.
	uint8_t pu8Dummy[0x1F];
    ReadReg(0x00, 0x1F, pu8Dummy);

    //6. Read the status registers(addr. 0x00, 0x01, 0x02)to detect possible errors and validate status. If any error exists, read again to ensure its validity.
    WriteReg(0x1C, 0x01, 0x00);//7. Write MODE(1:0) = "01" register (addr. 0x1C) to apply the configuration and enable the memory integrity check. In this mode configuration registers can only be read (except MODE(1:0) register, which is always accessible).(S.40)  

    return 0;
}

/*uint8_t iCHTP::readReg(uint8_t reg)
{
	openReg(reg);
	uint8_t reading; 					// holds byte of read data
	Wire.requestFrom(_addr, 1); 		// Request 1 byte from open register
	if (Wire.available() == 0){
		reading = 0;
	}
	else {
		reading = Wire.read();
	}
	
	return reading;
}*/

uint8_t iCHTP::ReadReg(uint8_t u8Cmd, uint8_t u8Len, uint8_t au8_RegsReturn[])   //ReadFromIcHtp
{
	SwitchToI2cChan(u8Channel);
//DEBUG("fertig: SwitchToI2cChan(" + (String)u8Channel + ")" );
	openReg(u8Cmd);
//DEBUG("fertig: openReg(" + (String)u8Cmd + ")" );


	Wire.requestFrom((int)_addr, (int)u8Len); 		// Request u8Len bytes from open register
//DEBUG("fertig: Wire.requestFrom((int)" + (String)_addr + ", " + (String)u8Len +")" );


	if (Wire.available() == 0)
	{
		return 1;//timeout oder anderer Fehler
	}
	else 
	{
		for(uint8_t u8Cnt = 0; u8Cnt < u8Len; u8Cnt++)
		{
//DEBUG("starte: au8_RegsReturn[" + (String)u8Cnt + "] = Wire.read();" );
			au8_RegsReturn[u8Cnt] = Wire.read();
//DEBUG("fertig: au8_RegsReturn[" + (String)u8Cnt + "] = Wire.read();" );
		}
	}
	return 0;
}

/*uint8_t iCHTP::writeReg(uint8_t reg, uint8_t data)
{
uint8_t bError	
	Wire.beginTransmission(_addr);		// Open Device
	Wire.write(reg);						// Point to register
	Wire.write(data);						// Write data to register 
	bError = Wire.endTransmission();				// Relinquish bus control	
		
}*/

uint8_t iCHTP::WriteReg(uint8_t u8Cmd, uint8_t u8Data0, uint8_t u8Data1)
{
uint8_t u8Error;
	SwitchToI2cChan(u8Channel);
	Wire.beginTransmission(_addr);		// Open Device	
  	Wire.write (u8Cmd);
  	Wire.write (u8Data0);
  	Wire.write (u8Data1);
	u8Error = Wire.endTransmission();				// Relinquish bus control	Rückgabewert: Error (=0, wenn alles in Ornung ist)  
	return u8Error;
}

void iCHTP::openReg(uint8_t reg)
{
	Wire.beginTransmission(_addr); 			// Connect to iCHTP
	Wire.write(reg); 						// point to specified register
	Wire.endTransmission(); 				// Relinquish bus control	
}
