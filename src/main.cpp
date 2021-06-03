
#include <SPI.h>
#include <EEPROM.h>

#include "Display.h"
#include "Setup.h"
#include "functions.h"
#include "Service.h"

#include "global_vars.h"
#include "Uart.h"

void setup() 
{
  //Serial.begin(115200);
  Serial.begin(38400);
  Serial1.begin(115200, 134217756U, P12, P14, false, 20000UL);
  EEPROM.begin(eepromSize);

  delay(1000);

  DEBUG((String)"\nSketchname: " + (__FILE__) + "\nBuild: " + (__TIMESTAMP__));
  DEBUG("System Clock: " + (String) ESP.getCpuFreqMHz() + " MHz");
  DEBUG("Running on core " + (String) xPortGetCoreID());

  delay(30);
  SystemSetup();
 
  delay(30);
  TecDac.Configure();
  TecAdc.begin();

  TecAdc.WritConfigReg( ADS1015_REG_CONFIG_OS_BUSY |
                        ADS1015_REG_CONFIG_MUX_SINGLE_0|
                        ADS1015_REG_CONFIG_PGA_6_144V|
                        ADS1015_REG_CONFIG_MODE_CONTIN|
                        ADS1015_REG_CONFIG_DR_3300SPS|
                        ADS1015_REG_CONFIG_CMODE_WINDOW|
                        ADS1015_REG_CONFIG_CPOL_ACTVHI|
                        ADS1015_REG_CONFIG_CLAT_LATCH|
                        ADS1015_REG_CONFIG_CQUE_NONE );
 
  gstAvailableIchtp.uiWord = 0;
  
  for(uint8_t u8LasNo=0; u8LasNo < (NO_OF_LASERS + NO_OF_HEATERS); u8LasNo++)
  {
    if (LaserDriver[u8LasNo].Configure(I2C_LAS_ADDRESS[u8LasNo],I2C_LAS_CHANNEL[u8LasNo])==0) 
    {
      gstAvailableIchtp.uiWord |= (uint16_t)((uint16_t)1<<(uint8_t)(u8LasNo));   
//DEBUG("LaserDriver[" + (String)u8LasNo + "] ist vorhanden" );
    }
    else
    {
//DEBUG("LaserDriver[" + (String)u8LasNo + "] ist NICHT vorhanden" );
    }
  }
  guiCurrentADChannel = 0;

  /*xTaskCreate(InternalTrigger_Task,         // Function to implement the task
                          "InternalTrigger_Task",       // Name of the task
                          10000,                        // Stack size in words
                          NULL,                         // Task input parameters
                          2,                            // Priority of the task
                          NULL);                         // Task handle
*/
//  xTaskCreatePinnedToCore(CycleTimer_Task,  "CycleTimer_Task", 10000, NULL,  1,  NULL, 0);
  xTaskCreate(CycleTimer_Task,  "CycleTimer_Task", 10000, NULL,  5,  NULL);


  InitExternTrigger(gucTriggerFunction);
}

void loop() 
{
unsigned char ucCntOvertemp = 0;
unsigned char uiCntTecVolt = 0;
#define PERIOD_TO_ADC 40 //20 entspricht 40ms pro Zyklus, nach 6 Zyklen ist Abfrage aller ADC fertig; Jede AD-Abfrage dauert 7,5ms
static uint8_t u8CntToAdc = 0;
delay(1);

  if (Serial.available())
    {
      UartReceiveOrTransmit();
//digitalWrite(GPIO_TEST_PIN, HIGH);
//DEBUG("UartReceiveOrTransmit");
    }
    //else
    {  
      if(FLAG_CHANGE_SETUP_REQUIRED)
      {
          FLAG_CHANGE_SETUP_REQUIRED=(bool)0;
          ReadSetupFromEeprom();
//DEBUG("FLAG_CHANGE_SETUP_REQUIRED");
      }
      else 
      { 
//digitalWrite(GPIO_TEST_PIN, LOW);//wird noch gesendet
//delayMicroseconds(10);
//digitalWrite(GPIO_TEST_PIN, HIGH);//wird noch gesendet

        if(FLAG_SAVE_SETUP_REQUIRED)
        {
          FLAG_SAVE_SETUP_REQUIRED=(bool)0;
          SaveSetupToEeprom();
//DEBUG("FLAG_SAVE_SETUP_REQUIRED");
        }
        else   
        {
//digitalWrite(GPIO_TEST_PIN, LOW);//nicht gesendet
          {
            if(FLAG_SET_I_LAS_REQUIRED)
            {
              if(bSetAllLaser)
              {
                gucLaserToSet--;
                if(0==gucLaserToSet) bSetAllLaser = (bool)0;
              }
              if(0==bSetAllLaser) FLAG_SET_I_LAS_REQUIRED=(bool)0;
              SetLaserDriver();
//DEBUG("FLAG_SET_I_LAS_REQUIRED");
            }
            else      
            {
//digitalWrite(GPIO_TEST_PIN, LOW);//nicht gesendet
              if(FLAG_SET_TEMP_REQUIRED)
              {
                  FLAG_SET_TEMP_REQUIRED=(bool)0;
                  TecDac.SetDac(TEMP_SET, gstDAC[TEMP_SET].uiWord);
//DEBUG("FLAG_SET_TEMP_REQUIRED");
              }
              else
              {
                if(FLAG_SET_DAC_B_REQUIRED)
                {
                    FLAG_SET_DAC_B_REQUIRED=(bool)0;
                    TecDac.SetDac(DAC_B, gstDAC[DAC_B].uiWord);
//DEBUG("FLAG_SET_DAC_B_REQUIRED");
                }
                else
                {
                  if(FLAG_SET_DAC_C_REQUIRED)
                  {
                      FLAG_SET_DAC_C_REQUIRED=(bool)0;
                      TecDac.SetDac(DAC_C, gstDAC[DAC_C].uiWord);
                  }
                  else
                  {
                    if(FLAG_SET_DAC_D_REQUIRED)
                    {
                        FLAG_SET_DAC_D_REQUIRED=(bool)0;
                        TecDac.SetDac(DAC_D, gstDAC[DAC_D].uiWord);
                    }
                    else
                    {
//digitalWrite(GPIO_TEST_PIN, LOW);//nicht gesendet

                      if(FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED)
                      {
                        FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED=(bool)0;
                        digitalWrite(GPIO_TRIGGER_OUT, LOW);        //Set Off
                        InitExternTrigger(gucTriggerFunction);
                      }
                      else
                      {
                        if(FLAG_READ_ADC_AND_SWITCH_REQUIRED)//wird von CycleTimer_Task ausgelöst
                        {
//DEBUG("FLAG_READ_ADC_AND_SWITCH_REQUIRED");
                            FLAG_READ_ADC_AND_SWITCH_REQUIRED=(bool)0;
                            if(u8CntToAdc++ < PERIOD_TO_ADC)
                            {
                              ReadSwitch();//Taster auslesen, um eventuell einzuschalten
                            }
                            else
                            {
                              u8CntToAdc = 0;
                              ReadAdc();//ADC: TEC-Controller Werte für Temperatur sowie Strom und Spannung am TEC sowie sämtliche Laserwerte lesen
                            
                              if (gstTecTemperature.uiWord > OVERTEMPERATURE)//Übertemperatur überschritten: Leite Ausschaltvorgang ein
                              {
                                if(ucCntOvertemp >3)//Wenn Übertremperatgur dreimal hintereinander unterschritten wurde, dann schalte alles aus
                                {
                                  gucDeviceState = STATE_OVERTEMP;
                                  gucDeviceStateShadow = gucDeviceState;
                                  GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_HI, LOW);   //Laser ausschalten
                                  GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_LO, LOW);   //Laser ausschalten
                                  GpioExpanderIc91.digitalWrite(IO_EN_HEATER_PWR, LOW);     //Heizwiderstand ausschalten
                                  GpioExpanderIc91.digitalWrite(IO_EN_TEC, LOW);            //Peltier-Element ausschalten
                                }
                                else
                                {
                                  ucCntOvertemp ++;
                                }
                              }
                              else
                              {
                                //if(gucDeviceState == STATE_OVERTEMP) gucDeviceState = OFF;
                                ucCntOvertemp = 0;
                              }
                                  
                              if (gucDeviceState==STATE_HEATING)//Warte bis Differenz zwischen Soll- und Isttemperatur Schwellwert unterschritten hat und schalte dann zu STATE_READY
                              {
																uint32_t u32SetTempCmp 	= (uint32_t)gstDAC[TEMP_SET].uiWord * 26;
																uint32_t u32ReadTempCmp = (uint32_t)gstTecTemperature.uiWord * 55;
																uint32_t u32TempDiff = abs(u32ReadTempCmp - u32SetTempCmp);
																if(u32TempDiff < 20000) //4000 entsprechen etwa 0,1K
                                {
                                  if((uiCntTecVolt++) > DELAY_AFTER_READY_TEC_VOLTAGE) 
                                  {
                                    gucDeviceState = STATE_READY;//kritische TEC-Spannung lange genug unterschritten: Gib Laser frei
                                    gucDeviceStateShadow = gucDeviceState;
                                  }
                                }
                                else
                                {
                                  uiCntTecVolt = 0;//kritische TEC-Spannung überschritten: Fang von vorne an zu zählen
                                }
                              }
                              else
                              {
                                uiCntTecVolt = 0;
                              }
                            }
                        }
                        else
                        {
                          if(FLAG_BLINK_LED_REQUIRED)//wird von Timer 4 ausgelöst
                          {
//DEBUG("FLAG_BLINK_LED_REQUIRED");
                              FLAG_BLINK_LED_REQUIRED=(bool)0;
                              BlinkLed();
                          }
                          else
                          {
                            if(FLAG_READ_ALL_ICHTP_REGISTERS)
                            {
                              LaserDriver[gu8_NoOfIchtpToRead].ReadReg(0x00, 0x1F, au8_I2cReadData);
                              FLAG_READ_ALL_ICHTP_REGISTERS=(bool)0;
                              for(uint8_t u8Cnt=0; u8Cnt<31; u8Cnt++)
                              {
                                au8_IchtpRegisters[u8Cnt]=au8_I2cReadData[u8Cnt] ;
                              }
                            }
                            else
                            {
                              if(FLAG_SET_DCO_REQUIRED)
                              {
                                uint8_t u8LasNo = LASER_OF_DCO[gu8DcoToSet];
                                if((gstAvailableIchtp.uiWord & ((uint16_t)((uint16_t)1<<(uint8_t)(u8LasNo)))) > 0)//Nur, wenn der iC-HTP am I2C-Bus angeschlossen ist
                                {
                                  LaserDriver[u8LasNo].WriteReg(0x1C, 0x02, 0x00);//2. Write MODE(1:0) = "10" register (addr. 0x1C) to enable the configuration mode.
                                  LaserDriver[u8LasNo].WriteReg(0x1B, gu8Dco[gu8DcoToSet], 0x00);// 
                                  LaserDriver[u8LasNo].WriteReg(0x1C, 0x01, 0x00);//7. Write MODE(1:0) = "01" register (addr. 0x1C) to apply the configuration and enable the memory integrity check. In this mode configuration registers can only be read (except MODE(1:0) register, which is always accessible).(S.40)
                                }
                                if(bSetAllDco)
                                {
                                  if(0==gu8DcoToSet) bSetAllDco = (bool)0; else gu8DcoToSet--;
                                }
                                if(0==bSetAllDco) FLAG_SET_DCO_REQUIRED=(bool)0;
//DEBUG("FLAG_SET_DCO_REQUIRED");
                              }
                              else
                              {
                                  //Kein Flag gesetzt
                              }
                            }
                          }
                        }                            
                      }
                    }
                  }
                }
              }                        
            }                
          }              
          
        }          
      }
    }
  }