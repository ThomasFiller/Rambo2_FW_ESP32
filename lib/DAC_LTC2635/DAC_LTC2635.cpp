#include "DAC_LTC2635.h"
#include <Wire.h>
#include "globalconsts.h"

LTC2635::LTC2635(uint8_t addr, uint8_t u8I2cChannel)
{
  	_addr = addr;
	u8Channel = u8I2cChannel;
}

void LTC2635::SwitchToI2cChan(uint8_t ucChannel) //Schaltet auf einen der 3 I2C-Kan�le durch Multiplexen des SCL-Signals
{
  if(ValBit(ucChannel, 0)) digitalWrite(GPIO_SELECT_I2C_2, HIGH); 	else digitalWrite(GPIO_SELECT_I2C_2, LOW);
  if(ValBit(ucChannel, 1)) digitalWrite(GPIO_SELECT_I2C, HIGH); 	else digitalWrite(GPIO_SELECT_I2C, LOW);
}

void LTC2635::Configure()
{
  SwitchToI2cChan(u8Channel);
	Wire.begin(); // Join I2C bus

//DEBUG("ConfigureDac(); u8Channel=" + (String)u8Channel);
  Wire.beginTransmission(_addr);    // Open Device
  //Wire.write(0x66);  //Select Internal Reference (Power Up Reference)
  Wire.write(0x77);  //Select External Reference (Power Down Internal Reference)
  Wire.write(0x77);  //dummy
  Wire.write(0x77);  //dummy
  uint8_t u8Error = Wire.endTransmission();       // Relinquish bus control 
  if(u8Error) 
  {
DEBUG_F("Fehler beim DAC-Schreiben:"+(String)(u8Error)+ ";   I2C_ERROR_ACK= " + (String)I2C_ERROR_ACK);
  }
  SetDac(TEMP_SET , 0);
  SetDac(DAC_B , 0);
  SetDac(DAC_C , 0);
  SetDac(DAC_D , 0);
}

void LTC2635::SetDac(uint8_t ucDacAddress , uint16_t ui16DacValue)
{
  typUnsignedWord stDacValue;
  SwitchToI2cChan(u8Channel);
  stDacValue.uiWord = ui16DacValue;
  //DEBUG("SetDac(" +  (String)(ucDacAddress) +"," +  (String)(ui16DacValue) + "); " + "stDacValue.stBytes.ucHighByte=" + (String)(stDacValue.stBytes.ucHighByte)+ ";     stDacValue.stBytes.ucLowByte=" + (String)(stDacValue.stBytes.ucLowByte));
  uint8_t u8Error;
  uint8_t u8_DacAdressByte = 0x20;      //Write to Input Register n, Update (Power Up) All
  u8_DacAdressByte |= ucDacAddress;
//DEBUG_F("SetDac("+(String)(ucDacAddress)+ ", " + (String)ui16DacValue + "); _addr="+ (String)_addr  + "; u8_DacAdressByte="+ (String)u8_DacAdressByte );

  Wire.beginTransmission(_addr);    // Open Device
  Wire.write(u8_DacAdressByte);            
  Wire.write(stDacValue.stBytes.ucHighByte); 
  Wire.write(stDacValue.stBytes.ucLowByte);
  u8Error = Wire.endTransmission();                     // Relinquish bus control 
  if(u8Error) 
  {
DEBUG("Fehler beim DAC-Schreiben:"+(String)(u8Error)+ ";   I2C_ERROR_ACK= " + (String)I2C_ERROR_ACK);
  }
}