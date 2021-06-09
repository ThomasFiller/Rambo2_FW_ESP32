#include "functions.h"
#include "Setup.h"
#include "global_vars.h"

uint8_t Reg0x1A[5]   =       {0x66, 0x44, 0x44, 0x44, 0x44};
uint8_t BaseReg0x10[5]   =   {0x90, 0x90, 0xB0, 0xD0, 0xF0};

void IRAM_ATTR TriggerInput(void)
{
static bool bLaserOn;  
  if((gucTriggerFunction & 0x07)==3) //state triggered
  {
    if (digitalRead(GPIO_TRIGGER_IN)) bLaserOn = true; else bLaserOn= false;
  }
  digitalWrite(GPIO_TRIGGER_OUT, bLaserOn); 
  bLaserOn = !bLaserOn;  

}

void InitExternTrigger(uint8_t u8TriggerFunction)
{
//DEBUG("gucTriggerFunction= "  + (String)gucTriggerFunction);
  if(gucTriggerFunction & 0x08) //Pullup einschalten?
  {
    GpioExpanderIc91.ModifyBuffer(IO_TRIGGER_nPULLUP, LOW);
  }
  else 
  {
    GpioExpanderIc91.ModifyBuffer(IO_TRIGGER_nPULLUP, HIGH);
  }
  GpioExpanderIc91.SendBufferToI2c();

  if(!(gucTriggerFunction & 0x07))//Interner Trigger?
  {
    double dFrequency = (double) gstInternalTriggerFrequency.uiWord;
    if(dFrequency > 1)    
      ledcSetup(GPIO_TRIGGER_OUT, dFrequency, 8); 
    else 
      ledcSetup(GPIO_TRIGGER_OUT, 100, 8);//100Hz

    ledcAttachPin(GPIO_TRIGGER_OUT, GPIO_TRIGGER_OUT);
    ledcWrite(GPIO_TRIGGER_OUT, 127);//PWM-Output für internen Trigger
    //delay(100);
//DEBUG("gstInternalTriggerFrequency HighByte= " + (String)gstInternalTriggerFrequency.stBytes.ucHighByte + "; LowByte= " + (String)gstInternalTriggerFrequency.stBytes.ucLowByte + "; word=" + (String)gstInternalTriggerFrequency.uiWord + "; dFrequency= " + (String)dFrequency);
  }
  else
  {
    ledcDetachPin(GPIO_TRIGGER_OUT);
    pinMode(GPIO_TRIGGER_OUT, OUTPUT);
  }
  
  switch ((u8TriggerFunction & 0x07))
  {
    case 0://internal Trigger aktiv => kein Interrupt aktivieren
      detachInterrupt(GPIO_TRIGGER_IN);
      break;
    case 1:
      attachInterrupt(GPIO_TRIGGER_IN, TriggerInput, RISING);
      break;
    case 2:
      attachInterrupt(GPIO_TRIGGER_IN, TriggerInput, FALLING);
      break;
     case 3:
      attachInterrupt(GPIO_TRIGGER_IN, TriggerInput, CHANGE);
      break;
    default:
      detachInterrupt(GPIO_TRIGGER_IN);
      break;
  }
}


void CycleTimer_Task(void * pvParameters)
{
//DEBUG_F("Running on core " + (String) xPortGetCoreID());
 static unsigned char ucCntBlinkLed=0;
  while(true)
  {
    if (ucCntBlinkLed++ > 8) {FLAG_BLINK_LED_REQUIRED = (bool)1; ucCntBlinkLed=0;}
    FLAG_READ_ADC_AND_SWITCH_REQUIRED = (bool)1;
    vTaskDelay(10);
  }
}

/**
 * Legt fest, ob ein Laser blinkt, an oder aus ist
 * @param u8Laser: Nummer des Lasers von 0..11
 * @param u8LedMode: 0 => permanent aus; 1 => blinken; 2 => alt. blinken; 3 => permanent an
 */
void SetLaserMode(uint8_t u8Laser, uint8_t u8LaserMode)
{
  gucEnLas[u8Laser]=u8LaserMode;
  uint8_t u8PullEnLas;
  uint8_t u8Prg0Las;
  if(ValBit(u8LaserMode, 0)) u8PullEnLas=HIGH; else u8PullEnLas=LOW;
  if(ValBit(u8LaserMode, 1)) u8Prg0Las=HIGH; else u8Prg0Las=LOW;

  if(IC94 == IC_LAS[u8Laser])
  {
DEBUG("IC94 SetLaserMode(" + (String)u8Laser + ", " + (String)u8LaserMode + "); u8PullEnLas=" + (String)u8PullEnLas + "; u8Prg0Las=" +(String)u8Prg0Las+ "; uIO_PULL_EN_LAS[u8Laser]="+ (String)IO_PULL_EN_LAS[u8Laser]+ "; IO_PRG0_LAS[u8Laser]="+ (String)IO_PRG0_LAS[u8Laser]);
    GpioExpanderIc94.ModifyBuffer(IO_PULL_EN_LAS[u8Laser], u8PullEnLas);
    GpioExpanderIc94.ModifyBuffer(IO_PRG0_LAS   [u8Laser], u8Prg0Las);
    GpioExpanderIc94.SendBufferToI2c();
  }
  else
  {
DEBUG("IC91 SetLaserMode(" + (String)u8Laser + ", " + (String)u8LaserMode + "); u8PullEnLas=" + (String)u8PullEnLas + "; u8Prg0Las=" +(String)u8Prg0Las+ "; uIO_PULL_EN_LAS[u8Laser]="+ (String)IO_PULL_EN_LAS[u8Laser]+ "; IO_PRG0_LAS[u8Laser]="+ (String)IO_PRG0_LAS[u8Laser]);
    GpioExpanderIc91.ModifyBuffer(IO_PULL_EN_LAS[u8Laser], u8PullEnLas);
    GpioExpanderIc91.ModifyBuffer(IO_PRG0_LAS   [u8Laser], u8Prg0Las);
    GpioExpanderIc91.SendBufferToI2c();
  }
}

void SetLaserDriver300mA(unsigned char ucDriverAddress ,typUnsignedWord uiDriverValue)
{
DEBUG("SetLaserDriver300mA(" + (String)ucDriverAddress + (String)uiDriverValue.uiWord + ")" );
  pinMode(GPIO_SPI_nCS, OUTPUT);
  pinMode(GPIO_SPI_SCK, OUTPUT);
  pinMode(GPIO_SPI_MOSI, OUTPUT_OPEN_DRAIN);


  pinMode(GPIO_SPI_MOSI, OUTPUT_OPEN_DRAIN);
  unsigned char aucSendByte[2];
  aucSendByte[0] = (ucDriverAddress<<4);
  aucSendByte[0] |= (0x0F & (uiDriverValue.stBytes.ucHighByte>>4));
  aucSendByte[1] = (0xF0 & (uiDriverValue.stBytes.ucHighByte<<4)) | (0x0F & (uiDriverValue.stBytes.ucLowByte>>4));
  
  digitalWrite(GPIO_SPI_nCS, LOW);		// Port   aus (nSYNC)
  
  for(unsigned char ucTmp=0; ucTmp<2; ucTmp++)
  {
    for(unsigned char ucBitCnt = 0; ucBitCnt<8; ucBitCnt++)
    {
      digitalWrite(GPIO_SPI_SCK, LOW);        // Port   aus (SCLK)
      if(aucSendByte[ucTmp] & (0x01<<(7-ucBitCnt)))
      {
        digitalWrite(GPIO_SPI_MOSI, HIGH);         // Port an (DIN)
      }
      else
      {
        digitalWrite(GPIO_SPI_MOSI, LOW);       // Port aus (DIN)       
      }
      delayMicroseconds(1);
      digitalWrite(GPIO_SPI_SCK, HIGH);        // Port   an (Rising edge of SCLK => Date->Reg)
      delayMicroseconds(1);
    }
  }
  digitalWrite(GPIO_SPI_nCS, HIGH);        // Port   an (nSYNC) 
  digitalWrite(GPIO_SPI_MOSI, HIGH);        // Port an (DIN)
}

void SwitchToI2cChan(uint8_t ucChannel) //Schaltet auf einen der 3 I2C-Kan�le durch Multiplexen des SCL-Signals
{
  if(ValBit(ucChannel, 0)) digitalWrite(GPIO_SELECT_I2C_2, HIGH); 	else digitalWrite(GPIO_SELECT_I2C_2, LOW);
  if(ValBit(ucChannel, 1)) digitalWrite(GPIO_SELECT_I2C, HIGH); 	else digitalWrite(GPIO_SELECT_I2C, LOW);
}

void SetLaserDriver()
{
    //if((gucLaserToSet< 0x0A) || (gucLaserToSet> 0x0B))//Ist das ein iC-HTP am i2c-Bus oder ein AD-Treiber am SPI?
    if((gucLaserToSet< 0x0E))//Ist das ein iC-HTP am i2c-Bus oder ein AD-Treiber am SPI?
    {//iC-HTP am i2c-Bus
      uint8_t u8RegAddr;
      uint8_t u8Reg0x1E, u81stLaserToSet, u82ndLaserToSet, u8Crng1, u8Crng2;

        LaserDriver[gucLaserToSet].WriteReg(0x1C, 0x02);//2. Write MODE(1:0) = "10" register (addr. 0x1C) to enable the configuration mode.
        //uint8_t u8RegAddr = 0x13 + ((gucLaserToSet & 0x01) * 5);//Geradzahlige Laser werden auf Register 0x13, ungeradzahlige auf Register 0x18 programmiert;
        if ((gucLaserToSet & 0x01) || (gucLaserToSet>(NO_OF_LASERS-1)))//richtigen Kanal w�hlen; bei Heaters ist Kanal immer u8RegAddr=0x13
        {//ungeradzahlig
          u81stLaserToSet = gucLaserToSet;
          u82ndLaserToSet = gucLaserToSet-1;
          u8RegAddr = 0x13;
        }
        else
        {//geradzahlig
          u81stLaserToSet = gucLaserToSet+1;
          u82ndLaserToSet = gucLaserToSet;
          u8RegAddr = 0x18;
        }
        
        u8Crng1 = (0x03 - (gstIlas[u81stLaserToSet].stBytes.ucHighByte >> 2));
        u8Crng2 = (0x03 - (gstIlas[u82ndLaserToSet].stBytes.ucHighByte >> 2));

        u8Reg0x1E = REG_0x1E_OFFSET | (u8Crng2<<4) | u8Crng1;
        
        //LaserDriver[gucLaserToSet].WriteReg(u8RegAddr, (REFREG_OFFSET | stDriverValue.stBytes.ucHighByte) , stDriverValue.stBytes.ucLowByte);//Laserstrom �ber Referenzspannung einstellen; siehe Tabelle 36 auf Seite 20   
        LaserDriver[gucLaserToSet].WriteRegTwoBytes(u8RegAddr, (REFREG_OFFSET | (gstIlas[gucLaserToSet].stBytes.ucHighByte & 0x03)) , gstIlas[gucLaserToSet].stBytes.ucLowByte);//Laserstrom �ber Referenzspannung einstellen; siehe Tabelle 36 auf Seite 20   
        LaserDriver[gucLaserToSet].WriteReg(0x1E, u8Reg0x1E);//Laser Current Range einstellen   
        LaserDriver[gucLaserToSet].WriteReg(0x1C, 0x01);//7. Write MODE(1:0) = "01" register (addr. 0x1C) to apply the configuration and enable the memory integrity check. In this mode configuration registers can only be read (except MODE(1:0) register, which is always accessible).(S.40)
    }
    else //AD-Treiber 300mA am SPI
    {
      unsigned char ucDriverAddress;
      if(gucLaserToSet == 0x0E)       ucDriverAddress = SPI_ADDRESS_LAS_DRV_10;
        else                            ucDriverAddress = SPI_ADDRESS_LAS_DRV_11;
      SetLaserDriver300mA(ucDriverAddress ,gstIlas[gucLaserToSet]);
    }
}

void ReadAndConfigureTecAdc()//ADC 4 channel
{
#define TEC_ADC_TEMPERATURE     ADS1015_REG_CONFIG_OS_BUSY      |\
                        ADS1015_REG_CONFIG_MUX_SINGLE_0 /*hier ist der Kanal*/ |\
                        ADS1015_REG_CONFIG_PGA_6_144V   |\
                        ADS1015_REG_CONFIG_MODE_CONTIN  |\
                        ADS1015_REG_CONFIG_DR_3300SPS   |\
                        ADS1015_REG_CONFIG_CMODE_WINDOW |\
                        ADS1015_REG_CONFIG_CPOL_ACTVHI  |\
                        ADS1015_REG_CONFIG_CLAT_LATCH   |\
                        ADS1015_REG_CONFIG_CQUE_NONE

#define TEC_ADC_VTEC_diff       ADS1015_REG_CONFIG_OS_BUSY      |\
                        ADS1015_REG_CONFIG_MUX_DIFF_2_3 /*hier ist der Kanal*/ |\
                        ADS1015_REG_CONFIG_PGA_6_144V   |\
                        ADS1015_REG_CONFIG_MODE_CONTIN  |\
                        ADS1015_REG_CONFIG_DR_3300SPS   |\
                        ADS1015_REG_CONFIG_CMODE_WINDOW |\
                        ADS1015_REG_CONFIG_CPOL_ACTVHI  |\
                        ADS1015_REG_CONFIG_CLAT_LATCH   |\
                        ADS1015_REG_CONFIG_CQUE_NONE

static uint16_t u8ConfigReg = 0;

  switch(u8ConfigReg)// & 0xF0)//Command
  {
    case TEC_ADC_TEMPERATURE:
      u8ConfigReg = TEC_ADC_VTEC_diff; 
      gstTecTemperature.uiWord = TecAdc.ReadConversionReg();
      break;
          
    case TEC_ADC_VTEC_diff:
      u8ConfigReg = TEC_ADC_TEMPERATURE; 
      gstTecVoltage.uiWord = TecAdc.ReadConversionReg();
      break;
          
    default:
      u8ConfigReg = TEC_ADC_TEMPERATURE; 
  }  
  TecAdc.WritConfigReg(u8ConfigReg);
}

bool bUngerade(int zahl)
{
	return zahl & 1;  
}

void SaveAdcDataOfOneChannel(uint8_t u8Configuration, uint8_t u8Channel)
{
uint8_t u8MapLocation, u8AdLocation;
  if((bUngerade(u8Channel)) || (u8Channel > (NO_OF_LASERS-1)))
  {
    u8MapLocation = 0x01;
    u8AdLocation = 3;
  }
  else
  {
    u8MapLocation = 0x10;   
    u8AdLocation = 5;
  }
  if(u8Configuration == LDA)
  {
    if((au8_I2cReadData[1] & u8MapLocation)) bMapc[u8Channel] = (bool)1; //Laser 2 eingeschaltet
                                    else bMapc[u8Channel] = (bool)0;     //Laser 2 nicht eingeschaltet
  }   
  
  if((u8Configuration != LDA) || (bMapc[u8Channel]==1) || (gucEnLas[u8Channel]==0))//�ndere den LDA-Wert nur, wenn der Laser nicht im Blinkmodus oder derzeit an ist
  {      
    gstAdcValue[u8Channel][u8Configuration].stBytes.ucHighByte = au8_I2cReadData[u8AdLocation];
    gstAdcValue[u8Channel][u8Configuration].stBytes.ucLowByte = au8_I2cReadData[u8AdLocation+1];
  }
 
  if(u8Configuration == I_LAS)
  {
    if((bMapc[u8Channel]    ==0)&& gucEnLas[u8Channel]    ==0)  gstAdcValue[u8Channel]  [I_LAS].uiWord = 0;        //Eanable Pin Laser 2 ist aus aber Laser ist nicht aus => Laser blinkt
  }  
}

void ReadAndConfigureAdc(uint8_t u8ConfigurationActualMeasurement, uint8_t u8ConfigurationNextMeasurement)
{
  for (uint8_t u8Cnt=0; u8Cnt < (NO_OF_LASERS + NO_OF_HEATERS); u8Cnt++)
  {
    uint16_t u16CurrentIchtp = ((uint16_t)((uint16_t)1<<(uint8_t)(u8Cnt)));
    if((gstAvailableIchtp.uiWord & u16CurrentIchtp) > 0)//nur wenn iC-HTP angeschlossen ist und funktioniert
    {
      LaserDriver[u8Cnt].ReadReg(0x00, 8, au8_I2cReadData);
      if(u8Cnt < NO_OF_LASERS)
      {   
        SaveAdcDataOfOneChannel(u8ConfigurationActualMeasurement, u8Cnt);//Laser2
        u8Cnt++;
      }
      SaveAdcDataOfOneChannel(u8ConfigurationActualMeasurement, u8Cnt);//Laser1
  
      //N�chste AD-Wandlung vorbereiten:
      LaserDriver[u8Cnt].WriteReg(0x1C, 0x02);//2. Write MODE(1:0) = "10" register (addr. 0x1C) to enable the configuration mode.
      LaserDriver[u8Cnt].WriteReg(0x1A, Reg0x1A[u8ConfigurationNextMeasurement]);//CMES
      LaserDriver[u8Cnt].WriteReg(0x10, BaseReg0x10[u8ConfigurationNextMeasurement] + 0x07);//ADCC1
      LaserDriver[u8Cnt].WriteReg(0x15, BaseReg0x10[u8ConfigurationNextMeasurement] + OFFSET_REG0x15[u8Cnt]);//ADCC2
      LaserDriver[u8Cnt].WriteReg(0x1C, 0x01);//7. Write MODE(1:0) = "01" register (addr. 0x1C) to apply the configuration and enable the memory integrity check. In this mode configuration registers can only be read (except MODE(1:0) register, which is always accessible).(S.40)
    }
  }
}

bool RegulationSourceVoltage(uint8_t u8FirstEmitterNo, uint8_t u8LastEmitterNo)
{
bool bReturnValue=(bool)0;
  if((unsigned char)(GpioExpanderIc91.digitalRead(IO_EN_PWR[DCO[u8FirstEmitterNo]])))//nur wenn das entsprechende Source, welches eventuell geregelt werden soll, eingeschaltet ist
  {
    unsigned int uiMinVoltage = 0xFFFF;
    unsigned int uiDiffVoltage;
    bool bRegulationRequest = (bool)0;//Wenn Laser eingeschaltet aber enable-Bit nicht gesetzt ist, dann befindet er sich im Blinkmodus und ist gerade aus. Wenn keiner eingeschaltet und enabled ist => nicht nachregeln!
    for(uint8_t u8CntLaser = u8FirstEmitterNo; u8CntLaser < (u8LastEmitterNo+1); u8CntLaser++)
    {
      if((gstAvailableIchtp.uiWord & ((uint16_t)((uint16_t)1<<(uint8_t)(u8CntLaser)))) > 0)//Nur, wenn der iC-HTP am I2C-Bus angeschlossen ist
      {
        if((gucEnLas[u8CntLaser]!=0))//geregelt wird nur, wenn Laser nicht aus ist
        {
          uiDiffVoltage = gstAdcValue[u8CntLaser][VBL].uiWord - gstAdcValue[u8CntLaser][LDA].uiWord;
          if (uiDiffVoltage < uiMinVoltage) uiMinVoltage = uiDiffVoltage;
          bRegulationRequest = (bool)1;//mindestens ein Laser ist eingeschaltet und enable-Bit ist gesetzt => nachregeln!
        }
      }
    }
    
    if(bRegulationRequest)// && (!bOneLaserIsBlinking))
    {
      bool bRegulationRequired =(bool)0;
      if (uiMinVoltage < DCO_REG_THRES) 
      {
        if (gu8Dco[DCO[u8FirstEmitterNo]] > 0) 
        {
          gu8Dco[DCO[u8FirstEmitterNo]]--;
          bRegulationRequired = (bool)1;
        }
      };
      if (uiMinVoltage > (DCO_REG_THRES + DCO_REG_HYSTERESIS)) 
      {
        if (gu8Dco[DCO[u8FirstEmitterNo]] < 63) 
        {
          gu8Dco[DCO[u8FirstEmitterNo]]++;
          bRegulationRequired = (bool)1;
        }
      };
      if(bRegulationRequired)
      {
        gu8DcoToSet = DCO[u8FirstEmitterNo];
        FLAG_SET_DCO_REQUIRED = (bool)1;   
        bReturnValue = (bool)1;
      }
    };
  };
  return bReturnValue;
}

void ReadAdc()
{
#ifdef REGULATE_POWERSOURCE
bool bRegulationInProgress;
#endif

   switch(guiCurrentADChannel)
  {
    case I_LAS: //Strom durch die einzelnen Laser
      ReadAndConfigureAdc( I_LAS, MDK);
      guiCurrentADChannel = MDK;      
    break;
    
    case MDK: //Temperatur und Spannung am Peltier-Element mit ADC Ads1015 lesen 
      ReadAndConfigureAdc( MDK, VB_VDD);
      guiCurrentADChannel = VB_VDD;      
    break;
    
    case VB_VDD: //Versorgungsspannungen Digitalelektronik iC-HTP
      ReadAndConfigureAdc(VB_VDD, VBL);
      guiCurrentADChannel = VBL; 
    break;
    
    case VBL:  //Versorgungsspannungen Lasertreiber
//Die Konfiguration zur LDA-Messung f�hrt zu Belastung der Laserquelle
      ReadAndConfigureAdc(VBL, LDA);//Die Konfiguration zur LDA-Messung f�hrt zu Belastung der Laserquelle
      guiCurrentADChannel = LDA;      
    break;

    case LDA: //Flussspannungen an den einzelnen Lasern
      ReadAndConfigureAdc(LDA, I_LAS);
      guiCurrentADChannel = EVALUATION_SOURCEVOLTAGE;      
    break;
    
    case EVALUATION_SOURCEVOLTAGE: //�berpr�fe, ob alle Versorgungsspannungen der Lasertreiber mindestens ca. 1V �ber dem Maximum aller Flusspannungen liegen
      guiCurrentADChannel = BOARD_TEMPERATURE_AND_HUMIDITY;  
#ifdef REGULATE_POWERSOURCE
      bRegulationInProgress =  RegulationSourceVoltage(0, 3);
      bRegulationInProgress |= RegulationSourceVoltage(4, 9);
      RegulationSourceVoltage(10, 11);
      RegulationSourceVoltage(12, 13);
      if(bRegulationInProgress)
      {
        gucDeviceState = STATE_REGULATION_IN_PROGRESS;
      }
      else
      {
        gucDeviceState = gucDeviceStateShadow;
      }
#endif
    break;   
    
    case BOARD_TEMPERATURE_AND_HUMIDITY:       
      guiCurrentADChannel = TEC; 
      TempHumSensor.ReadTempAndHum(&gstBoardTemperature.uiWord, &gstBoardHumidity.uiWord);
    break;
    
    case TEC: //Temperatur und Spannung am Peltier-Element mit ADC Ads1015 lesen 
      ReadAndConfigureTecAdc();
      guiCurrentADChannel = I_LAS;      
    break;
  
    default:
      guiCurrentADChannel = I_LAS;
  }
}
