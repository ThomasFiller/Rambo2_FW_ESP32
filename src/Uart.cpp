#include "global_vars.h"
#include "functions.h"
#include "Uart.h"

void UartSendErrorMessage(unsigned char ucDataDirection, unsigned char ucErrorCode)
{
  unsigned char pucStringToSend[30]; 
  //pucStringToSend[0]= gucMyUartAddress + 0x80;//1. Byte: Address of Board
  pucStringToSend[0]= DEFAULT_UART_ADDRESS_TRANSMIT + 0x80;//1. Byte: Address of Board
  pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
  pucStringToSend[2]= UART_ERROR;
  pucStringToSend[3]= ucErrorCode;//3. Byte -> 1.Datenbyte
  UartTransmitString(pucStringToSend);
}

void UartReceiveOrTransmit(void)
{
    uint8_t au8UartData[120]= {0};
    uint8_t u8RwAddress, u8LenOfFrame, u8UartCommand, ucChecksum = 0;
    String uart_data;
    Serial.readBytes(&u8RwAddress, 1);
    Serial.readBytes(&u8LenOfFrame, 1);
    Serial.readBytes(&u8UartCommand, 1);
    Serial.readBytes(au8UartData, u8LenOfFrame - 4);
    Serial.readBytes(&ucChecksum, 1);
//DEBUG("u8RwAddress " + (String)u8RwAddress + "; u8LenOfFrame " + (String)u8LenOfFrame + "; u8UartCommand " + (String)u8UartCommand+ "; au8UartData[0] " + (String)au8UartData[0]+ "; au8UartData[1] " + (String)au8UartData[1]);

    if ((u8RwAddress & 0x7F) == DEFAULT_UART_ADDRESS_RECEIVE) //is it my Address?
    {
      //ClrBitMASK(TIM4->CR1, TIM4_CR1_CEN); //Timer4 f�r zyklische AD-Wandlung disabled	
      ucChecksum = u8RwAddress ^ u8LenOfFrame  ^ u8UartCommand ^ ucChecksum;
      for(uint8_t u8Cnt = 0; u8Cnt < (u8LenOfFrame-4); u8Cnt++)
        ucChecksum ^= au8UartData[u8Cnt];

      if(ucChecksum)
        UartSendErrorMessage(DataDirection, ERROR_CHECKSUM);
      else
        if(ValBit(u8RwAddress, 7))
          ReplyData(u8UartCommand);
        else
          SetParameter(u8UartCommand, au8UartData, u8LenOfFrame);
      
      
        //SetBitMask(TIM4->CR1, TIM4_CR1_CEN); //Timer4 f�r zyklische AD-Wandlung enabled	
    }
}


void UartTransmitString(unsigned char *pucSendString)
{//pucSendString[1] has to contain length of complete frame
unsigned char ucCnt;
unsigned char ucChecksum=0;
uint8_t datalen = pucSendString[1];
  for (ucCnt=0; ucCnt < datalen-1;ucCnt++)
  {
    ucChecksum = ucChecksum ^ pucSendString[ucCnt];
  }
  pucSendString[datalen - 1] = ucChecksum;
  Serial.write(pucSendString, datalen);
  Serial.flush();
//DEBUG("gesendet: " + (String)pucSendString[0] + "; "+ (String)pucSendString[1] + "; "+ (String)pucSendString[2] + "; "+ (String)pucSendString[3] + "; "+ (String)pucSendString[4] + "; "+ (String)pucSendString[5] + "; ");

  // To clear the UART TX FIFO, we have to fill it with 10 char (default). Otherwise, the ESP32 will not send out the ACK immediatly (it's a dirty  ;) )
  //#define ACK                           0x06    // Acknowledge
  //byte ack[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ACK};
  //Serial.write(ack, 10);
  //Serial.flush();
}

void ReplyData(unsigned char ucCmd)//DataDirection == 1
{
unsigned char pucStringToSend[120]; 
  //pucStringToSend[0] = gucMyUartAddress + 0x80;//1. Byte: Address of Board
  pucStringToSend[0] = DEFAULT_UART_ADDRESS_TRANSMIT + 0x80;//1. Byte: Address of Board
  pucStringToSend[2] = ucCmd;
  switch(ucCmd)// & 0xF0)//Command
  {
    case UART_INTTRIG_FREQ://Frequenz mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet; bei Data=0 wird interner Trigger abgeschaltet
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstInternalTriggerFrequency.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstInternalTriggerFrequency.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;
    case UART_INTTRIG_RAT://Tastverh�ltnis mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= 0;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= 0;//4. Byte -> 2.Datenbyte
      break;
    case UART_TRIG_FUNCTION://liest Triggerfunktion: 0-interner Trigger, 1-extern rising edge, 2-extern falling edge, 3-extern state
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gucTriggerFunction;//3. Byte -> 1.Datenbyte
      break;
    case UART_SETUP_NO://liest eine der vier Setupnummern, die sich parrallel auch mit dem Schalter einstellen und �ber die LEDs ablesen lassen
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gucSetupNo;//3. Byte -> 1.Datenbyte
      break;
      
    case UART_STORAGE://16 Reservedaten zum Abspeichern im EEPROM (z.Bsp. Tmax, Tmin�); 1. Byte: Adresse (0..15); zweites Byte: Datum
      pucStringToSend[1] = NO_OF_STORAGE_BYTES+4;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      for (unsigned char ucCnt=0; ucCnt<NO_OF_STORAGE_BYTES; ucCnt++)     pucStringToSend[ucCnt+3]= EEPROM.read(eeucStorageBytes[ucCnt]);//3.-18. Byte -> Datenbytes
      break;
      
    case UART_DEVICE_STATE://OFF=0; HEATING=1; READY=2; ON=3; OVERTEMP=4
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gucDeviceState;//3. Byte -> 1.Datenbyte
      break;
      
  //Umweltdaten uC-Board	      
    case UART_BOARD_TEMPERATURE:
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstBoardTemperature.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstBoardTemperature.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte      
      break;
    case UART_BOARD_HUMIDITY:
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstBoardHumidity.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstBoardHumidity.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;
      
    case UART_READ_ALL://Liest alle Daten, die f�r zyklische Anzeige ben�tigt werden (Messdaten)
      pucStringToSend[1] = 18;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      //UART_VTEC
      pucStringToSend[3]= gstTecVoltage.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstTecVoltage.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      //UART_ITEC
      pucStringToSend[5]= 0;//3. Byte -> 1.Datenbyte
      pucStringToSend[6]= 0;//4. Byte -> 2.Datenbyte
      //UART_TMP
      pucStringToSend[7]= gstTecTemperature.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[8]= gstTecTemperature.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      //UART_DEVICE_STATE
      pucStringToSend[9]= gucDeviceState;
      //UART_EN_TEC
      pucStringToSend[10]= (uint8_t) (GpioExpanderIc91.digitalRead(IO_EN_TEC)); //3. Byte -> 1.Datenbyte
      //UART_BOARD_TEMPERATURE
      pucStringToSend[11]= gstBoardTemperature.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[12]= gstBoardTemperature.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte 
      //UART_BOARD_HUMIDITY
      pucStringToSend[13]= gstBoardHumidity.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[14]= gstBoardHumidity.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;
      
    case UART_READ_ICHTP:
      pucStringToSend[1] = 35;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      for(uint8_t u8Cnt=0; u8Cnt<31; u8Cnt++)
      {
        pucStringToSend[u8Cnt+3]= au8_IchtpRegisters[u8Cnt];
      }
      break;
      
    case UART_READ_ADC_LASER:
      pucStringToSend[1] = 104;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      for(uint8_t u8Cnt1=0; u8Cnt1 < NO_OF_LASERS; u8Cnt1++)
      {
        for(uint8_t u8Cnt2=0; u8Cnt2<5; u8Cnt2++)
        {
          uint8_t u8Index = 2*(u8Cnt1*5+u8Cnt2);
          pucStringToSend[u8Index+3]= gstAdcValue[u8Cnt1][u8Cnt2].stBytes.ucHighByte;
          pucStringToSend[u8Index+4]= gstAdcValue[u8Cnt1][u8Cnt2].stBytes.ucLowByte;//          
        }
      }
      break;
          
    case UART_READ_ADC_HEATER:
      pucStringToSend[1] = 44;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      for(uint8_t u8Cnt1=0; u8Cnt1 < NO_OF_HEATERS; u8Cnt1++)
      {
        for(uint8_t u8Cnt2=0; u8Cnt2<5; u8Cnt2++)
        {
          uint8_t u8Index = 2*(u8Cnt1*5+u8Cnt2);
          pucStringToSend[u8Index+3]= gstAdcValue[u8Cnt1+NO_OF_LASERS][u8Cnt2].stBytes.ucHighByte;
          pucStringToSend[u8Index+4]= gstAdcValue[u8Cnt1+NO_OF_LASERS][u8Cnt2].stBytes.ucLowByte;//          
        }
      }
      break;
                  
  //Lasertreiber und Teiber f�r Heizwiderstand		
    case UART_EN_LAS_HEAT_PWR://Spannungsregler f�r die Versorgung der Laserstromquellen eigeschaltet? (0-aus; 1-an)
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= (unsigned char) (GpioExpanderIc91.digitalRead(IO_EN_LASER_PWR_HI)) ;//3. Byte -> 1.Datenbyte
      break;
      
    case UART_DCO_HI:
    case UART_DCO_LO:
    case UART_DCO_RES0:
    case UART_DCO_RES1:
    case UART_DCO_RES2:
    case UART_DCO_RES3:
    case UART_MAX_TEC_HEAT_CURRENT:
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gu8Dco[ucCmd & 0x0F] & 0x3F;//3. Byte -> 1.Datenbyte    
      break;  
      
    case UART_AVAILABLE_ICHTP:
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstAvailableIchtp.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstAvailableIchtp.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;

    case UART_I_LAS0://Lesen des eingestellten des Laserstromes Treiber 0
    case UART_I_LAS1://Lesen des eingestellten des Laserstromes Treiber 1
    case UART_I_LAS2://Lesen des eingestellten des Laserstromes Treiber 2
    case UART_I_LAS3://Lesen des eingestellten des Laserstromes Treiber 3
    case UART_I_LAS4://Lesen des eingestellten des Laserstromes Treiber 4
    case UART_I_LAS5://Lesen des eingestellten des Laserstromes Treiber 5
    case UART_I_LAS6://Lesen des eingestellten des Laserstromes Treiber 6
    case UART_I_LAS7://Lesen des eingestellten des Laserstromes Treiber 7
    case UART_I_LAS8://Lesen des eingestellten des Laserstromes Treiber 8
    case UART_I_LAS9://Lesen des eingestellten des Laserstromes Treiber 9
    case UART_I_LAS10://Lesen des eingestellten des Laserstromes Treiber 10 (300mA-Treiber f�r schnelle Schaltvorg�nge)   
    case UART_I_LAS11://Lesen des eingestellten des Laserstromes Treiber 11 (300mA-Treiber f�r schnelle Schaltvorg�nge)
    case UART_I_HEATER0:
    case UART_I_HEATER1:
    case UART_I_HEATER2:
    case UART_I_HEATER3://lesen des eingestellten Stroms am Heizwiderstand
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstIlas[ucCmd & 0x0F].stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstIlas[ucCmd & 0x0F].stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;

    case UART_EN_LAS0://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS1://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS2://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS3://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS4://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS5://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS6://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS7://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS8://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS9://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gucEnLas[ucCmd & 0x0F];//3. Byte -> 1.Datenbyte
      break;
                          
  //TEC-Control			//			
    case UART_EN_TEC://Peltier-Element eingeschaltet?
      pucStringToSend[1] = 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= (unsigned char)(GpioExpanderIc91.digitalRead(IO_EN_TEC));//3. Byte -> 1.Datenbyte
      break;
    case UART_VTEC://liest die aktuelle Spannung am Peltierelement
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstTecVoltage.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstTecVoltage.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;
    case UART_TMP://liest die aktuelle Temperatur
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstTecTemperature.stBytes.ucHighByte;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstTecTemperature.stBytes.ucLowByte;//4. Byte -> 2.Datenbyte
      break;
    case UART_TMP_SET://Solltemperatur lesen
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstDAC[TEMP_SET].stBytes.ucHighByte ;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstDAC[TEMP_SET].stBytes.ucLowByte ;//4. Byte -> 2.Datenbyte
      break;
    case UART_DAC_B://Freien DAC-Kanal lesen
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstDAC[DAC_B].stBytes.ucHighByte ;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstDAC[DAC_B].stBytes.ucLowByte ;//4. Byte -> 2.Datenbyte
      break;
    case UART_DAC_C://Freien DAC-Kanal lesen
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstDAC[DAC_C].stBytes.ucHighByte ;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstDAC[DAC_C].stBytes.ucLowByte ;//4. Byte -> 2.Datenbyte
      break;
    case UART_DAC_D://Freien DAC-Kanal lesen
      pucStringToSend[1] = 6;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)
      pucStringToSend[3]= gstDAC[DAC_D].stBytes.ucHighByte ;//3. Byte -> 1.Datenbyte
      pucStringToSend[4]= gstDAC[DAC_D].stBytes.ucLowByte ;//4. Byte -> 2.Datenbyte
      break;
      
    default:
      pucStringToSend[1]= 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes) 
      pucStringToSend[2]= UART_ERROR;
      pucStringToSend[3]= ERROR_WRONG_CMD;
      break;
  }
  //delayMicroseconds(1000);
  UartTransmitString(pucStringToSend);
}

void SetParameter(unsigned char ucCmd, unsigned char *pucData, unsigned char ucLenOfFrame) //DataDirection==0
{
unsigned char pucStringToSend[10]; 
  //pucStringToSend[0]=(gucMyUartAddress);  //1. Byte: 7.Bit=1 (read); other Bytes = Address of this Device  
  pucStringToSend[0]= DEFAULT_UART_ADDRESS_TRANSMIT;  //1. Byte: 7.Bit=1 (read); other Bytes = Address of this Device  
  pucStringToSend[1]=4;//2. Byte: Len  
  pucStringToSend[2]=ucCmd; 

  switch(ucCmd)
  {
  //Allgemein; int. Trigger; Setup   
    case UART_INTTRIG_FREQ://Frequenz mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet; bei Data=0 wird interner Trigger abgeschaltet
      gstInternalTriggerFrequency.stBytes.ucHighByte = pucData[0];
      gstInternalTriggerFrequency.stBytes.ucLowByte  = pucData[1];
//DEBUG("gstInternalTriggerFrequency HighByte= " + (String)gstInternalTriggerFrequency.stBytes.ucHighByte + "; LowByte= " + (String)gstInternalTriggerFrequency.stBytes.ucLowByte + "; word=" + (String)gstInternalTriggerFrequency.uiWord + ");
      FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED = (bool)1;
      break;
    case UART_INTTRIG_RAT://Tastverh�ltnis mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet
      break;
    case UART_TRIG_FUNCTION:
      gucTriggerFunction = pucData[0];    
      FLAG_CHANGE_TRIGGER_FUNCTION_REQUIRED = (bool)1;
      break;    
    case UART_SETUP_NO://liest eine der Setupnummern, die sich parrallel auch mit dem Schalter einstellen und �ber die LEDs ablesen lassen
      if(pucData[0]<NO_OF_SETUPS) gucSetupNo = pucData[0]; else gucSetupNo = 0;
      FLAG_CHANGE_SETUP_REQUIRED = (bool)1;
      break;
    case UART_SAVE_SETUP://speichert aktuelles Setup und die Setupnummer
      FLAG_SAVE_SETUP_REQUIRED = (bool)1;
      break;
    
    case UART_STORAGE://16 Reservedaten zum Abspeichern im EEPROM (z.Bsp. Tmax, Tmin�); 1. Byte: Adresse (0..15); zweites Byte: Datum
      EEPROM.write(eeucStorageBytes[pucData[0]] , pucData[1]);
      EEPROM.commit();
      break;   

    case UART_ICHTP_NO_TO_READ://Legt fest, von welchem iC-HTTP alle Daten gelesen werden sollen und veranlasst Lesevorgang
      gu8_NoOfIchtpToRead = pucData[0];
      FLAG_READ_ALL_ICHTP_REGISTERS = (bool)1;
      break;
                  
  //Lasertreiber			//			
    case UART_EN_LAS_HEAT_PWR://Spannungsregler f�r die Versorgung der Laserstromquellen und Heizwiderst�nde eigeschaltet? (0-aus; 1-an)
      GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_HI,pucData[0]);
      GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_LO,pucData[0]);
      GpioExpanderIc91.digitalWrite(IO_EN_HEATER_PWR,pucData[0]);
      if(1==pucData[0])
      {
        bPwrSourcesJustEnabled = (bool)1;
        guiCurrentADChannel = I_LAS;//StateMachine von ReadAdc() zur�cksetzen, damit das Auslesen der iC-HTP erst m�glichst sp�t erfolgt
        gucDeviceState = STATE_ON;
        gucDeviceStateShadow = gucDeviceState;
      }
      else
      {
        if (GpioExpanderIc91.digitalRead(IO_EN_TEC))
        {
          gucDeviceState = STATE_HEATING;
          gucDeviceStateShadow = gucDeviceState;
        }
        else
        {
          gucDeviceState = STATE_OFF;
          gucDeviceStateShadow = gucDeviceState;
        }
      }
      break;
      
    case UART_DCO_HI:
    case UART_DCO_LO:
    case UART_DCO_RES0:
    case UART_DCO_RES1:
    case UART_DCO_RES2:
    case UART_DCO_RES3:
    case UART_MAX_TEC_HEAT_CURRENT:
      gu8DcoToSet = ucCmd & 0x0F;
      gu8Dco[gu8DcoToSet] = pucData[0];
      FLAG_SET_DCO_REQUIRED = (bool)1;      
      break;
      
    case UART_I_LAS0://Schreiben des eingestellten des Laserstromes Treiber 0
    case UART_I_LAS1://Schreiben des eingestellten des Laserstromes Treiber 1
    case UART_I_LAS2://Schreiben des eingestellten des Laserstromes Treiber 2
    case UART_I_LAS3://Schreiben des eingestellten des Laserstromes Treiber 3
    case UART_I_LAS4://Schreiben des eingestellten des Laserstromes Treiber 4
    case UART_I_LAS5://Schreiben des eingestellten des Laserstromes Treiber 5
    case UART_I_LAS6://Schreiben des eingestellten des Laserstromes Treiber 6
    case UART_I_LAS7://Schreiben des eingestellten des Laserstromes Treiber 7
    case UART_I_LAS8://Schreiben des eingestellten des Laserstromes Treiber 8
    case UART_I_LAS9://Schreiben des eingestellten des Laserstromes Treiber 9   
    case UART_I_LAS10://Schreiben des eingestellten des Laserstromes Treiber 10 (300mA-Treiber f�r schnelle Schaltvorg�nge)   
    case UART_I_LAS11://Schreiben des eingestellten des Laserstromes Treiber 11 (300mA-Treiber f�r schnelle Schaltvorg�nge)
    case UART_I_HEATER0:
    case UART_I_HEATER1:
    case UART_I_HEATER2:
    case UART_I_HEATER3://Strom am Heizwiderstand einstellen
      gucLaserToSet = ucCmd & 0x0F;
      gstIlas[gucLaserToSet].stBytes.ucHighByte=pucData[0];
      gstIlas[gucLaserToSet].stBytes.ucLowByte=pucData[1];
      FLAG_SET_I_LAS_REQUIRED = (bool)1;
      break;

    case UART_EN_LAS0://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS1://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS2://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS3://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS4://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS5://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS6://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS7://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS8://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
    case UART_EN_LAS9://Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
      SetLaserMode(ucCmd & 0x0F, pucData[0]);
      break;

                          
  //TEC-Control			//			
    case UART_EN_TEC://Peltier-Element ein- oder ausschalten
      if(pucData[0]==1) 
      {
        GpioExpanderIc91.digitalWrite(IO_EN_TEC, HIGH);//Peltier-Element einschalten
        gucDeviceState = STATE_HEATING; 
        gucDeviceStateShadow = gucDeviceState;
      }
      else 
      {
        GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_HI, LOW);//Laser ausschalten
        GpioExpanderIc91.digitalWrite(IO_EN_LASER_PWR_LO, LOW);//Laser ausschalten
        GpioExpanderIc91.digitalWrite(IO_EN_HEATER_PWR, LOW);//Heizwiderstand ausschalten
        GpioExpanderIc91.digitalWrite(IO_EN_TEC, LOW);//Peltier-Element ausschalten
        gucDeviceState = STATE_OFF;
        gucDeviceStateShadow = gucDeviceState;
      }
      break;
    case UART_TMP_SET://Solltemperatur einstellen
      gstDAC[TEMP_SET].stBytes.ucHighByte=pucData[0];
      gstDAC[TEMP_SET].stBytes.ucLowByte=pucData[1];
      FLAG_SET_TEMP_REQUIRED = (bool)1;
      break;
    case UART_DAC_B:
      gstDAC[DAC_B].stBytes.ucHighByte=pucData[0];
      gstDAC[DAC_B].stBytes.ucLowByte=pucData[1];
      FLAG_SET_DAC_B_REQUIRED = (bool)1;
      break;
    case UART_DAC_C:
      gstDAC[DAC_C].stBytes.ucHighByte=pucData[0];
      gstDAC[DAC_C].stBytes.ucLowByte=pucData[1];
      FLAG_SET_DAC_B_REQUIRED = (bool)1;
      break;
    case UART_DAC_D:
      gstDAC[DAC_D].stBytes.ucHighByte=pucData[0];
      gstDAC[DAC_D].stBytes.ucLowByte=pucData[1];
      FLAG_SET_DAC_B_REQUIRED = (bool)1;
      break;
    
    default:
      pucStringToSend[1]= 5;//1. Byte: Length of frame (5 bei einem Datenbyte; 6 bei 2 Datenbytes)  
      pucStringToSend[2]= UART_ERROR;
      pucStringToSend[3]= ERROR_WRONG_CMD;
      break;
  }
  UartTransmitString(pucStringToSend);
}