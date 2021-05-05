//#pragma SYMBOLS

#include "global_vars.h"

//*******************************Flag STM8 guiFlags
bool FLAG_CHANGE_SETUP_REQUIRED              ;
bool FLAG_SAVE_SETUP_REQUIRED                ;
bool FLAG_READ_ADC_AND_SWITCH_REQUIRED       ;

bool FLAG_SET_I_LAS_REQUIRED                 ;
bool FLAG_READ_ALL_ICHTP_REGISTERS           ;
bool FLAG_SET_DCO_REQUIRED                   ;

bool FLAG_SET_TEMP_REQUIRED                  ;
bool FLAG_SET_DAC_B_REQUIRED                 ;
bool FLAG_SET_DAC_C_REQUIRED                 ;
bool FLAG_SET_DAC_D_REQUIRED                 ;

bool FLAG_BLINK_LED_REQUIRED                 ;
bool FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED   ;

uint8_t gucSetupNo;
uint8_t gucTriggerFunction; //Triggerfunktion: 0-interner Trigger, 1-extern rising edge, 2-extern falling edge, 3-extern state
typUnsignedWord gstInternalTriggerFrequency;
uint8_t gucLaserToSet = 0;
uint8_t gu8DcoToSet = 0;
uint8_t au8_I2cData[4];
uint8_t au8_I2cReadData[0x20];
uint8_t au8_IchtpRegisters[0x20];
uint8_t gu8_NoOfIchtpToRead;
bool bSetAllLaser = (bool)1;
bool bSetAllDco = (bool)1;
bool bPwrSourcesJustEnabled=(bool)0;

uint8_t   IO_PULL_EN_LAS[NO_OF_LASERS]= {IO_PULL_EN_LAS0,IO_PULL_EN_LAS1,IO_PULL_EN_LAS2,IO_PULL_EN_LAS3,IO_PULL_EN_LAS4,IO_PULL_EN_LAS5,IO_PULL_EN_LAS6,IO_PULL_EN_LAS7,IO_PULL_EN_LAS8,IO_PULL_EN_LAS9,IO_PULL_EN_LAS10,IO_PULL_EN_LAS11 };
uint8_t   IO_PRG0_LAS   [NO_OF_LASERS]= {IO_PRG0_LAS0   ,IO_PRG0_LAS1   ,IO_PRG0_LAS2   ,IO_PRG0_LAS3   ,IO_PRG0_LAS4   ,IO_PRG0_LAS5   ,IO_PRG0_LAS6   ,IO_PRG0_LAS7   ,IO_PRG0_LAS8   ,IO_PRG0_LAS9   ,IO_PRG0_LAS10   ,IO_PRG0_LAS11 };
uint8_t   IC_LAS        [NO_OF_LASERS]= {IC_LAS0        ,IC_LAS1        ,IC_LAS2        ,IC_LAS3        ,IC_LAS4        ,IC_LAS5        ,IC_LAS6        ,IC_LAS7        ,IC_LAS8        ,IC_LAS9        ,IC_LAS10        ,IC_LAS11 };

//uint8_t              EN_LAS_PIN [NO_OF_LASERS]       = { EN_LAS0_PIN,  EN_LAS1_PIN,  EN_LAS2_PIN,  EN_LAS3_PIN,  EN_LAS4_PIN,  EN_LAS5_PIN,  EN_LAS6_PIN,  EN_LAS7_PIN,  EN_LAS8_PIN,  EN_LAS9_PIN };

uint8_t              I2C_LAS_ADDRESS[NO_OF_LASERS + NO_OF_HEATERS]   = {I2C_ADDRESS_LAS_DRV_E, I2C_ADDRESS_LAS_DRV_E, I2C_ADDRESS_LAS_DRV_D, I2C_ADDRESS_LAS_DRV_D, I2C_ADDRESS_LAS_DRV_C, I2C_ADDRESS_LAS_DRV_C, I2C_ADDRESS_LAS_DRV_B, I2C_ADDRESS_LAS_DRV_B, I2C_ADDRESS_LAS_DRV_A, I2C_ADDRESS_LAS_DRV_A, I2C_ADDRESS_HEAT_DRV_0, I2C_ADDRESS_HEAT_DRV_1, I2C_ADDRESS_HEAT_DRV_2, I2C_ADDRESS_HEAT_DRV_3};
uint8_t              I2C_LAS_CHANNEL[NO_OF_LASERS + NO_OF_HEATERS]   = {SCL1, SCL1, SCL1, SCL1, SCL1, SCL1, SCL2, SCL2, SCL2, SCL2, SCL3, SCL3, SCL3, SCL3}; 
uint8_t              OFFSET_REG0x15[NO_OF_LASERS + NO_OF_HEATERS]   = {0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x0F, 0x0F, 0x0F, 0x0F}; 

uint8_t              I2C_DCO_CHANNEL[2 + NO_OF_HEATERS]  = {SCL1, SCL1, SCL3, SCL3, SCL3, SCL3};
uint8_t              gu8Dco[2 + NO_OF_HEATERS + 1]           = {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,0x3F};
uint8_t              LASER_OF_DCO[2 + NO_OF_HEATERS]= {0, 4, 10, 11, 12, 13};

uint8_t              IO_EN_PWR[2 + NO_OF_HEATERS]={     IO_EN_LASER_PWR_HI,IO_EN_LASER_PWR_LO ,IO_EN_HEATER_PWR,IO_EN_HEATER_PWR,IO_EN_HEATER_PWR,IO_EN_HEATER_PWR};
uint8_t              DCO[NO_OF_LASERS + NO_OF_HEATERS] ={0,0,0,0,1,1,1,1,1,1,2,2,3,3};

iCHTP   LaserDriver[NO_OF_LASERS + NO_OF_HEATERS];
HDC2080 TempHumSensor(I2C_ADDRESS_TEMP_HUM_SENSOR, SCL1);
LTC2635 TecDac(I2C_ADDRESS_DAC1, I2C_CHAN_DAC);
Adafruit_ADS1015 TecAdc(I2C_ADDRESS_ADC1, I2C_CHAN_ADC);
PCF8575 GpioExpanderIc94(I2C_ADDRESS_GPIO_EXPANDER_UCBOARD_IC94, SCL1);
PCF8575 GpioExpanderIc91(I2C_ADDRESS_GPIO_EXPANDER_UCBOARD_IC91, SCL1);

typUnsignedWord	gstTecTemperature;
typUnsignedWord	gstTecVoltage;

typUnsignedWord gstAvailableIchtp;
typUnsignedWord	gstDAC[4];
typUnsignedWord	gstIlas[NO_OF_LASERS + NO_OF_HEATERS + NO_OF_300mALASERS];
uint8_t gucEnLas[NO_OF_LASERS];
typUnsignedWord gstBoardTemperature;
typUnsignedWord gstBoardHumidity;
typUnsignedWord	gstAdcValue[NO_OF_LASERS + NO_OF_HEATERS][5];
bool bMapc[NO_OF_LASERS + NO_OF_HEATERS];

typUnsignedWord gstTemperature;
typUnsignedWord gstItec;
typUnsignedWord gstVtec;
typUnsignedWord gstTec5V;

uint8_t DataDirection;

uint16_t guiCurrentADChannel;

uint8_t gucDeviceState=STATE_OFF;
uint8_t gucDeviceStateShadow=STATE_OFF;

volatile uint8_t i2c_err_save;   	// I2C1->SR2 copy in case of error

//EEPROM - Parameterablage
const int eeucSetupNo = 0;
const int eeucStorageBytes[NO_OF_STORAGE_BYTES] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
const int eeucCheckFirstCall={26};
const int eeucDacLowByte[NO_OF_DACS][NO_OF_SETUPS]={27,28,29,30,31,32,33,34};
const int eeucDacHighByte[NO_OF_DACS][NO_OF_SETUPS]={35,36,37,38,39,40,41,42};
const int eeucIlasLowByte[NO_OF_LASERS+NO_OF_HEATERS+NO_OF_300mALASERS][NO_OF_SETUPS]={43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74};
const int eeucIlasHighByte[NO_OF_LASERS+NO_OF_HEATERS+NO_OF_300mALASERS][NO_OF_SETUPS]={75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106};
const int eeucEnLas[NO_OF_LASERS][NO_OF_SETUPS]={107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126};
const int eeucIntTrigFreqLowByte[NO_OF_SETUPS]={127,128};
const int eeucIntTrigFreqHighByte[NO_OF_SETUPS]={129,130};
const int eeucTriggerFunction[NO_OF_SETUPS]={131,132};
const int eeucDco[2+NO_OF_HEATERS+1][NO_OF_SETUPS]={133,134,135,136,137,138,139, 140,141,142,143,144,145,146};

const size_t eepromSize = 150;