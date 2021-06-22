#ifndef __Global_Vars_H_included
#define __Global_Vars_H_included

#include "globalconsts.h"
#include "EEPROM.h"
#include "iCHTP.h"
#include "HDC2080.h"
#include "DAC_LTC2635.h"
#include "Adafruit_ADS1015.h"
#include "PCF8575.h"              // I/O-Port Expander

//*******************************Flag guiFlags
extern bool FLAG_CHANGE_SETUP_REQUIRED              ;
extern bool FLAG_SAVE_SETUP_REQUIRED                ;
extern bool FLAG_READ_ADC_AND_SWITCH_REQUIRED       ;

extern bool FLAG_SET_I_LAS_REQUIRED                 ;
extern bool FLAG_READ_ALL_ICHTP_REGISTERS           ;
extern bool FLAG_SET_DCO_REQUIRED                   ;

extern bool FLAG_SET_TEMP_REQUIRED                  ;
extern bool FLAG_SET_DAC_B_REQUIRED                 ;
extern bool FLAG_SET_DAC_C_REQUIRED                 ;
extern bool FLAG_SET_DAC_D_REQUIRED                 ;

extern bool FLAG_BLINK_LED_REQUIRED                 ;
extern bool FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED   ;

//extern uint8_t gucMyUartAddress;
extern uint8_t gucSetupNo;
extern uint8_t gucTriggerFunction; //Triggerfunktion: 0-interner Trigger, 1-extern rising edge, 2-extern falling edge, 3-extern state
extern typUnsignedWord gstInternalTriggerFrequency;
extern uint8_t gucLaserToSet;
extern uint8_t gu8DcoToSet;
extern uint8_t gucHeaterToSet;
extern uint8_t au8_I2cData[4];
extern uint8_t au8_I2cReadData[];
extern uint8_t au8_IchtpRegisters[];
extern uint8_t gu8_NoOfIchtpToRead;
extern bool bSetAllLaser;
extern bool bSetAllDco;
extern bool bPwrSourcesJustEnabled;

extern uint8_t   IO_PULL_EN_LAS[NO_OF_LASERS];
extern uint8_t   IO_PRG0_LAS   [NO_OF_LASERS];
extern uint8_t   IC_LAS        [NO_OF_LASERS];


extern uint8_t              I2C_LAS_ADDRESS[NO_OF_LASERS + NO_OF_HEATERS];
extern uint8_t              I2C_LAS_CHANNEL[NO_OF_LASERS + NO_OF_HEATERS];
extern uint8_t              OFFSET_REG0x15[NO_OF_LASERS + NO_OF_HEATERS]; 

extern uint8_t              I2C_DCO_CHANNEL[2 + NO_OF_HEATERS];
extern uint8_t              IO_EN_PWR [2 + NO_OF_HEATERS];
extern uint8_t              DCO[NO_OF_LASERS + NO_OF_HEATERS];
extern uint8_t              LASER_OF_DCO[2 + NO_OF_HEATERS];


extern uint8_t gu8Dco[2 + NO_OF_HEATERS + 1];

extern iCHTP   LaserDriver[];
extern HDC2080 TempHumSensor;
extern LTC2635 TecDac;
extern Adafruit_ADS1015 TecAdc;
extern Adafruit_ADS1015 LiaAdc;
extern PCF8575 GpioExpanderIc94;
extern PCF8575 GpioExpanderIc91;
extern PCF8575 GpioExpanderTia;
extern PCF8575 GpioExpanderLia;

extern uint8_t u8AvailableI2c;

extern typUnsignedWord gstAvailableIchtp;

extern typUnsignedWord	gstDAC[];		
extern typUnsignedWord	gstIlas[];	
extern unsigned char gucEnLas[];
extern typUnsignedWord gstBoardTemperature;
extern typUnsignedWord gstBoardHumidity;
extern typUnsignedWord	gstAdcValue[NO_OF_LASERS + NO_OF_HEATERS][5];
extern bool bMapc[NO_OF_LASERS + NO_OF_HEATERS];

extern typUnsignedWord	gstTecTemperature;
extern typUnsignedWord	gstTecVoltage;

extern typSigned32	gstLiaVoltage;

extern typUnsignedWord gstTiaDigital;
extern typUnsignedWord gstLiaDigital;
extern uint8_t u8LiaAnalogRange;
extern uint8_t u8LiaAnalogAvgDepth;

extern unsigned char DataDirection;
extern uint16_t guiCurrentADChannel;

extern unsigned char gucDeviceState;
extern unsigned char gucDeviceStateShadow;

extern volatile uint8_t i2c_err_save;   	// I2C1->SR2 copy in case of error

//EEPROM - Parameterablage
extern const int eeucSetupNo;
extern const int eeucStorageBytes[NO_OF_STORAGE_BYTES];
extern const int eeucCheckFirstCall;//= 0;

extern const int eeucDacLowByte[NO_OF_DACS][NO_OF_SETUPS];
extern const int eeucDacHighByte[NO_OF_DACS][NO_OF_SETUPS];

extern const int eeucIlasLowByte[NO_OF_LASERS+NO_OF_HEATERS+NO_OF_300mALASERS][NO_OF_SETUPS];
extern const int eeucIlasHighByte[NO_OF_LASERS+NO_OF_HEATERS+NO_OF_300mALASERS][NO_OF_SETUPS];
extern const int eeucEnLas[NO_OF_LASERS][NO_OF_SETUPS];

extern const int eeucIntTrigFreqLowByte[NO_OF_SETUPS];
extern const int eeucIntTrigFreqHighByte[NO_OF_SETUPS];
extern const int eeucTriggerFunction[NO_OF_SETUPS];

extern const int eeucDco[2 + NO_OF_HEATERS+1][NO_OF_SETUPS];

extern const int eeucTiaDigital[NO_OF_SETUPS];
extern const int eeucLiaDigitalLowByte[NO_OF_SETUPS];
extern const int eeucLiaDigitalHighByte[NO_OF_SETUPS];
extern const int eeucLiaAnalogRange[NO_OF_SETUPS];
extern const int eeucLiaAnalogAvgDepth[NO_OF_SETUPS];

extern const size_t eepromSize;

#endif