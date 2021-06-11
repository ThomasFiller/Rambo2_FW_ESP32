#include "global_vars.h"
#include "functions.h"

void GPIO_Setup(void);
void FirstCall();
void ReadSetupFromEeprom();//Parameter aus EEPROM einlesen
void SetupTempHumSensor();

void SystemSetup()
{
  GPIO_Setup();
  FLAG_CHANGE_SETUP_REQUIRED=(bool)1;
  if(!(EEPROM.read(eeucCheckFirstCall)==0x8F))
  {//erste Initialisierung EEPROM nach dem Programmieren
    FirstCall();
  }
  gucSetupNo = EEPROM.read(eeucSetupNo);
  ReadSetupFromEeprom();                                                        //Parameter aus EEPROM einlesen
  SetupTempHumSensor();
}

void SetupTempHumSensor()
{
  //--------------------TempHumSensorUcBoard-----------------------------------------------------------------------------------------------
  TempHumSensor.begin();// Initialize I2C communication
  TempHumSensor.reset();// Begin with a device reset
  
  // Set up the comfort zone
  TempHumSensor.setHighTemp(28);         // High temperature of 28C
  TempHumSensor.setLowTemp(22);          // Low temperature of 22C
  TempHumSensor.setHighHumidity(55);     // High humidity of 55%
  TempHumSensor.setLowHumidity(40);      // Low humidity of 40%
  
  // Configure Measurements
  TempHumSensor.setMeasurementMode(TEMP_AND_HUMID);  // Set measurements to temperature and humidity
  TempHumSensor.setRate(ONE_HZ);                     // Set measurement frequency to 1 Hz
  TempHumSensor.setTempRes(FOURTEEN_BIT);
  TempHumSensor.setHumidRes(FOURTEEN_BIT);
  
  //begin measuring
  TempHumSensor.triggerMeasurement();
}

void FirstCall()//wird nur beim ersten mal nach dem Programmieren aufgerufen, um EEPROM zu definieren
{
  EEPROM.write(eeucSetupNo , 0);
  for(gucSetupNo = 0; gucSetupNo<NO_OF_SETUPS; gucSetupNo++)
  {
    for (unsigned char ucCnt=1; ucCnt<NO_OF_DACS; ucCnt++)
    {  
      EEPROM.write(eeucDacLowByte[ucCnt][gucSetupNo] , 0);
      EEPROM.write(eeucDacHighByte[ucCnt][gucSetupNo], 0); 
    }
    EEPROM.write(eeucDacLowByte[TEMP_SET][gucSetupNo] , 0xb3);//23�C
    EEPROM.write(eeucDacHighByte[TEMP_SET][gucSetupNo], 0xb3);//23�C 
    for (unsigned char ucCnt=0; ucCnt<NO_OF_LASERS + NO_OF_HEATERS; ucCnt++)
    {  
      EEPROM.write(eeucIlasLowByte[ucCnt][gucSetupNo], 0);
      EEPROM.write(eeucIlasHighByte[ucCnt][gucSetupNo], 0); 
      if (ucCnt < NO_OF_LASERS) (EEPROM.write(eeucEnLas[ucCnt][gucSetupNo], 0)); 
    }
    for (unsigned char ucCnt=0; ucCnt<(2 + NO_OF_HEATERS); ucCnt++)
    {    
      EEPROM.write(eeucDco[ucCnt][gucSetupNo] , 63);
    }
    EEPROM.write(eeucDco[2 + NO_OF_HEATERS][gucSetupNo] , 48);

    EEPROM.write(eeucIntTrigFreqLowByte[gucSetupNo]  , (unsigned char) 100);
    EEPROM.write(eeucIntTrigFreqHighByte[gucSetupNo] ,  (unsigned char) 0);
    EEPROM.write(eeucTriggerFunction[gucSetupNo]  ,   (unsigned char) 0);
  }
  EEPROM.write(eeucStorageBytes[T_MIN] , 20 );
  EEPROM.write(eeucStorageBytes[T_MAX] , 31 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_0] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_1] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_2] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_3] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_4] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_5] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_6] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_7] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_8] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_LAS_9] , 190 );
  EEPROM.write(eeucStorageBytes[I_MAX_HEATER0] , 175 );
  EEPROM.write(eeucStorageBytes[I_MAX_HEATER1] , 175 );
  EEPROM.write(eeucStorageBytes[I_MAX_HEATER2] , 175 );
  EEPROM.write(eeucStorageBytes[I_MAX_HEATER3] , 175 );
  EEPROM.write(eeucStorageBytes[SER_NO] , 0 );
  EEPROM.write(eeucStorageBytes[NO_OF_GROUPS] , 1 );
  EEPROM.write(eeucStorageBytes[FIRST_DRV_1] , 0 );
  EEPROM.write(eeucStorageBytes[LAST_DRV_1] , 3 );
  EEPROM.write(eeucStorageBytes[FIRST_DRV_2] , 4 );
  EEPROM.write(eeucStorageBytes[LAST_DRV_2] , 5 );

  gucSetupNo = 0;
  EEPROM.write(eeucCheckFirstCall , 0x8F);  

  EEPROM.commit();  
}

void ReadSetupFromEeprom()//Parameter aus EEPROM einlesen
{
  for (unsigned char ucCnt=0; ucCnt<NO_OF_DACS; ucCnt++)
  {  
    gstDAC[ucCnt].stBytes.ucLowByte  = EEPROM.read(eeucDacLowByte[ucCnt][gucSetupNo]);
    gstDAC[ucCnt].stBytes.ucHighByte = EEPROM.read(eeucDacHighByte[ucCnt][gucSetupNo]); 
  }
  
  for (unsigned char ucCnt=0; ucCnt < (2 + NO_OF_HEATERS+1); ucCnt++)
  {    
    gu8Dco[ucCnt] = EEPROM.read(eeucDco[ucCnt][gucSetupNo]);
  }
  
  for (unsigned char ucCnt=0; ucCnt < (NO_OF_LASERS+NO_OF_300mALASERS+NO_OF_HEATERS); ucCnt++)
  {  
    gstIlas[ucCnt].stBytes.ucLowByte  = EEPROM.read(eeucIlasLowByte[ucCnt][gucSetupNo]);
    gstIlas[ucCnt].stBytes.ucHighByte = EEPROM.read(eeucIlasHighByte[ucCnt][gucSetupNo]); 
    if(ucCnt < (NO_OF_LASERS))
    {
      SetLaserMode(ucCnt, EEPROM.read(eeucEnLas[ucCnt][gucSetupNo])); //Nur Laser lassen sich direkt schalten
    }
    else
    {
      gucEnLas[ucCnt] = 0xFF;//Enabele-Pins aller HeaterDriver sind immer an
    }
  //DEBUG("ReadSetupFromEeprom; gstIlas[" + (String)ucCnt + "]= " + (String)gstIlas[ucCnt].uiWord);
  }

  gstInternalTriggerFrequency.stBytes.ucHighByte = EEPROM.read(eeucIntTrigFreqHighByte[gucSetupNo]);
  gstInternalTriggerFrequency.stBytes.ucLowByte  = EEPROM.read(eeucIntTrigFreqLowByte[gucSetupNo]);
//  DEBUG(" Von EEPROM gelesen: gstInternalTriggerFrequency.stBytes.ucHighByte= " + (String) gstInternalTriggerFrequency.stBytes.ucHighByte + "; gstInternalTriggerFrequency.stBytes.ucLowByte= " + (String) gstInternalTriggerFrequency.stBytes.ucLowByte + "; gucSetupNo= " +gucSetupNo);

  gucTriggerFunction    = EEPROM.read(eeucTriggerFunction[gucSetupNo]);
  //gucMyUartAddress      = eeucMyUartAddress;
  
  FLAG_SET_I_LAS_REQUIRED = (bool)1;
  FLAG_SET_TEMP_REQUIRED = (bool)1;
  FLAG_SET_DAC_B_REQUIRED = (bool)1;
  FLAG_SET_DAC_C_REQUIRED = (bool)1;
  FLAG_SET_DAC_D_REQUIRED = (bool)1;
  FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED = (bool)1;
  FLAG_SET_DCO_REQUIRED = (bool)1;
  gucLaserToSet = NO_OF_LASERS + NO_OF_HEATERS + NO_OF_300mALASERS;
  gu8DcoToSet = 2 + NO_OF_HEATERS; 
  bSetAllLaser = (bool)1;
  bSetAllDco = (bool)1;
}
  
void SaveSetupToEeprom(void)//EEPROM - Parameterablage
{
  EEPROM.write(eeucSetupNo , gucSetupNo);
  for (unsigned char ucCnt=0; ucCnt<NO_OF_DACS; ucCnt++)
  {  
    EEPROM.write(eeucDacLowByte[ucCnt][gucSetupNo]         , gstDAC[ucCnt].stBytes.ucLowByte);
    EEPROM.write(eeucDacHighByte[ucCnt][gucSetupNo]        , gstDAC[ucCnt].stBytes.ucHighByte); 
  }
  
  for (unsigned char ucCnt=0; ucCnt<(2 + NO_OF_HEATERS + 1); ucCnt++)
  {    
    EEPROM.write(eeucDco[ucCnt][gucSetupNo], gu8Dco[ucCnt]);
  }
  
  for (unsigned char ucCnt=0; ucCnt<(NO_OF_LASERS + NO_OF_HEATERS + NO_OF_300mALASERS); ucCnt++)
  {  
    EEPROM.write(eeucIlasLowByte[ucCnt][gucSetupNo]        , gstIlas[ucCnt].stBytes.ucLowByte);
    EEPROM.write(eeucIlasHighByte[ucCnt][gucSetupNo]       , gstIlas[ucCnt].stBytes.ucHighByte); 
    if (ucCnt < NO_OF_LASERS) EEPROM.write(eeucEnLas[ucCnt][gucSetupNo] , gucEnLas[ucCnt]); 
  DEBUG("SaveSetupToEeprom; gstIlas[" + (String)ucCnt + "]= " + (String)gstIlas[ucCnt].uiWord);
  }

  EEPROM.write(eeucIntTrigFreqLowByte[gucSetupNo]    ,   gstInternalTriggerFrequency.stBytes.ucLowByte);
  EEPROM.write(eeucIntTrigFreqHighByte[gucSetupNo]   ,   gstInternalTriggerFrequency.stBytes.ucHighByte);
  EEPROM.write(eeucTriggerFunction[gucSetupNo] , gucTriggerFunction);

  EEPROM.commit();
DEBUG("SaveSetupToEeprom; gstInternalTriggerFrequency= " + (String) gstInternalTriggerFrequency.uiWord + "; gstInternalTriggerFrequency.stBytes.ucHighByte= " + gstInternalTriggerFrequency.stBytes.ucHighByte+ "; gstInternalTriggerFrequency.stBytes.ucLowByte= " + gstInternalTriggerFrequency.stBytes.ucLowByte);
}

/**
  * @brief   Initialisiert die GPIOs
  * @param  None
  * @retval None
  */
void GPIO_Setup()
{
  //pinMode(GPIO_TEST_PIN, OUTPUT);
  //digitalWrite(GPIO_TEST_PIN, HIGH);

//******************************i2c
  pinMode(GPIO_I2C_SDA, OUTPUT_OPEN_DRAIN); 
  pinMode(GPIO_I2C_SCL, OUTPUT_OPEN_DRAIN); 
  
  pinMode(GPIO_SELECT_I2C, OUTPUT);
  pinMode(GPIO_SELECT_I2C_2, OUTPUT);
  //SELECT_I2C | SELECT_I2C_2 | Activ SCL
  //-----------|--------------|-----------
  //    0      |     X        | I2C_SCL_1         (I2C_SCL_2 = 1; I2C_SCL_3 = 1)
  //    1      |     0        | I2C_SCL_2         (I2C_SCL_1 = 1; I2C_SCL_3 = 1)
  //    1      |     1        | I2C_SCL_3         (I2C_SCL_1 = 1; I2C_SCL_2 = 1)
  
//********************************SPI
  pinMode(GPIO_SPI_nCS, OUTPUT);
  pinMode(GPIO_SPI_SCK, OUTPUT);
  pinMode(GPIO_SPI_MOSI, OUTPUT_OPEN_DRAIN);
  digitalWrite(GPIO_SPI_MOSI, HIGH);        // Port an (DIN)
 
//******************************Versorgung und Ansteuerung Laser-Driver
  GpioExpanderIc91.pinMode(IO_EN_LASER_PWR_HI, OUTPUT);
  GpioExpanderIc91.pinMode(IO_EN_LASER_PWR_LO, OUTPUT);

  for(uint8_t cnt=0; cnt<NO_OF_LASERS; cnt++)
  {
    if(IC94 == IC_LAS[cnt])
    {
      GpioExpanderIc94.pinMode(IO_PULL_EN_LAS[cnt], OUTPUT);
      GpioExpanderIc94.pinMode(IO_PRG0_LAS   [cnt], OUTPUT);
    }
    else
    {
      GpioExpanderIc91.pinMode(IO_PULL_EN_LAS[cnt], OUTPUT);
      GpioExpanderIc91.pinMode(IO_PRG0_LAS   [cnt], OUTPUT);
    }
  }

//******************************TEC-Controller Port
  GpioExpanderIc91.pinMode(IO_EN_TEC, OUTPUT);

//******************************Setup-Taster
  pinMode(GPIO_SWITCH, INPUT_PULLUP);
  
//******************************Trigger-Ausgang synchron zu internem Generator bzw. -Eingang zur externen Triggerung
//DEBUG("GPIO_TRIGGER_OUT = " + (String)GPIO_TRIGGER_OUT);
  pinMode(GPIO_TRIGGER_OUT, OUTPUT);
  pinMode(GPIO_TRIGGER_IN, INPUT);
  GpioExpanderIc91.pinMode(IO_TRIGGER_nPULLUP, OUTPUT);
  
//******************************Spannungsversorgung Heizwiderstand
  GpioExpanderIc91.pinMode(IO_EN_HEATER_PWR, OUTPUT);

//******************************Display-LED
  GpioExpanderIc91.pinMode(IO_LED1_BU, OUTPUT);
  GpioExpanderIc91.pinMode(IO_LED1_MINUS, OUTPUT);
  GpioExpanderIc91.pinMode(IO_LED1_PLUS, OUTPUT);

//*****************************TIA / LIA
  for (uint8_t u8CntTia = 0; u8CntTia < 5; u8CntTia++) GpioExpanderTia.pinMode(u8CntTia, OUTPUT);
  GpioExpanderTia.pinMode(5, INPUT);

  for (uint8_t u8CntLia = 0; u8CntLia < 16; u8CntLia++) GpioExpanderLia.pinMode(u8CntLia, OUTPUT);
  
//****************************Starte GPIO-Expander
  if (GpioExpanderIc94.begin() == 0) 
  {
    SetBit(u8AvailableI2c, BIT_IC94_AVAILABLE); 
DEBUG("GpioExpanderIc91 ist vorhanden" );
  }  
  
  if (GpioExpanderIc91.begin() == 0) 
  {
    SetBit(u8AvailableI2c, BIT_IC91_AVAILABLE); 
DEBUG("GpioExpanderIc91 ist vorhanden" );
  }

uint8_t u8Error;
  u8Error = GpioExpanderTia.begin();
  if (u8Error == 0) 
  {
    SetBit(u8AvailableI2c, BIT_TIA_AVAILABLE); 
DEBUG("TIA ist vorhanden" );
  }
  else
  {
DEBUG("TIA ist NICHT vorhanden; u8Error= " + (String)u8Error );
  }

  u8Error = GpioExpanderLia.begin();
  if (u8Error == 0) 
  {
    SetBit(u8AvailableI2c, BIT_LIA_DIGITAL_AVAILABLE);
DEBUG("LIA ist vorhanden" );
  }
  else
  {
DEBUG("LIA ist NICHT vorhanden" + (String)u8Error );
  }

  
  
  

  GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_HI, LOW);
  GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_LO, LOW);
  GpioExpanderIc91.ModifyBuffer(IO_EN_HEATER_PWR, LOW);
  GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, LOW);
  GpioExpanderIc91.ModifyBuffer(IO_LED1_PLUS, HIGH);
  GpioExpanderIc91.SendBufferToI2c();

//digitalWrite(GPIO_TRIGGER_OUT, LOW);        //Set Off
}
