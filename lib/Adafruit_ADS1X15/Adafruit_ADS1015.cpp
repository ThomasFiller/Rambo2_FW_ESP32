#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "Adafruit_ADS1015.h"
#include "globalconsts.h"

void Adafruit_ADS1015::SwitchToI2cChan() //Schaltet auf einen der 3 I2C-Kan�le durch Multiplexen des SCL-Signals
{
  if(ValBit(u8Channel, 0)) digitalWrite(GPIO_SELECT_I2C_2, HIGH); 	else digitalWrite(GPIO_SELECT_I2C_2, LOW);
  if(ValBit(u8Channel, 1)) digitalWrite(GPIO_SELECT_I2C, HIGH); 	else digitalWrite(GPIO_SELECT_I2C, LOW);
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
    @return the byte read
*/
/**************************************************************************/
static uint8_t i2cread(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library

    @param x byte to write
*/
/**************************************************************************/
static void i2cwrite(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register

    @param i2cAddress I2C address of device
    @param reg register address to write to
    @param value value to write to register
*/
/**************************************************************************/
uint8_t Adafruit_ADS1015::writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) 
{
uint8_t u8Error;
  SwitchToI2cChan();
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value >> 8));
  i2cwrite((uint8_t)(value & 0xFF));
  u8Error = Wire.endTransmission();
//DEBUG("u8Error @ i2cAddress " + (String)i2cAddress + " = " + (String)u8Error + "; u8Channel=" + (String)u8Channel);
  return u8Error;
}

/**************************************************************************/
/*!
    @brief  Read 16-bits from the specified destination register

    @param i2cAddress I2C address of device
    @param reg register address to read from

    @return 16 bit register value read
*/
/**************************************************************************/
uint16_t Adafruit_ADS1015::readRegister(uint8_t i2cAddress, uint8_t reg) 
{
  SwitchToI2cChan();
  Wire.beginTransmission(i2cAddress);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((i2cread() << 8) | i2cread());
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1015 class w/appropriate properties
    @param i2cAddress I2C address of device
    @param u8I2cChannel I2C-Kanal: einer von drei Kanälen
*/
/**************************************************************************/
Adafruit_ADS1015::Adafruit_ADS1015(uint8_t i2cAddress, uint8_t u8I2cChannel) 
{
  u8Channel = u8I2cChannel;
  m_i2cAddress = i2cAddress;
  m_conversionDelay = ADS1015_CONVERSIONDELAY;
  m_bitShift = 4;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
    @param i2cAddress I2C address of device
    @param u8I2cChannel I2C-Kanal: einer von drei Kanälen
*/
/**************************************************************************/
/*Adafruit_ADS1115::Adafruit_ADS1115(uint8_t i2cAddress, uint8_t u8I2cChannel) 
{
  u8Channel = u8I2cChannel;
  m_i2cAddress = i2cAddress;
  m_conversionDelay = ADS1115_CONVERSIONDELAY;
  m_bitShift = 0;
  m_gain = GAIN_TWOTHIRDS; // +/- 6.144V range (limited to VDD +0.3V max!) 
}*/

/**************************************************************************/
/*!
    @brief  Sets up the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void Adafruit_ADS1015::begin() { Wire.begin(); }

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range

    @param gain gain setting to use
*/
/**************************************************************************/
void Adafruit_ADS1015::setGain(adsGain_t gain) { m_gain = gain; }

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range

    @return the gain setting
*/
/**************************************************************************/
adsGain_t Adafruit_ADS1015::getGain() { return m_gain; }


/**************************************************************************/
/*!
    @brief  Schreibt in das Config Register 0x01
    @param value Wert, der ins Config Register geschrieben wird
    @return nichts
*/
/**************************************************************************/
uint8_t Adafruit_ADS1015::WritConfigReg(uint16_t value)
{
  return writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, value);
}


/**************************************************************************/
/*!
    @brief  Gibt den Wert der letzten AD-Wandlung zurück
    @param -
    @return Wert der letzten ADC-Conversion
*/
/**************************************************************************/
uint16_t Adafruit_ADS1015::ReadConversionReg()
{
  return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT);
}


/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
uint16_t Adafruit_ADS1015::readADC_SingleEnded(uint8_t channel) 
{
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::readADC_Differential_0_1() {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::readADC_Differential_2_3() {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

    @param channel ADC channel to use
    @param threshold comparator threshold
*/
/**************************************************************************/
void Adafruit_ADS1015::startComparator_SingleEnded(uint8_t channel,
                                                   int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1015_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH,
                threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

    @return the last ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::getLastConversionResults() {
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}
