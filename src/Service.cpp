#include "Service.h"
#include "global_vars.h"
#include "math.h"
#include "globalconsts.h"

#define LED_OFF		0
#define LED_RED 	1
#define LED_GREEN	2

void LedColor(uint8_t u8Color)
{
  static uint8_t u8ColorBuffer=10;
  if(u8Color != u8ColorBuffer)
  {
//DEBUG_F("LedColor(" + (String) u8Color + ")");
    if (LED_RED   == u8Color)   GpioExpanderIc91.ModifyBuffer(IO_LED1_PLUS , HIGH); else GpioExpanderIc91.ModifyBuffer(IO_LED1_PLUS , LOW);
    if (LED_GREEN == u8Color)   GpioExpanderIc91.ModifyBuffer(IO_LED1_MINUS, HIGH); else GpioExpanderIc91.ModifyBuffer(IO_LED1_MINUS, LOW);
    GpioExpanderIc91.SendBufferToI2c();
    u8ColorBuffer = u8Color;
  }
}

void BlinkLed()
{
  static unsigned char ucCntFlash = 9;
//DEBUG_F("BlinkLed; gucDeviceState=" + (String)gucDeviceState);
  switch(gucDeviceState)
    {
      case STATE_OVERTEMP:
        if (ucCntFlash++ > 3) 
        {
          LedColor(LED_RED);//RGB(TRUE,FALSE,FALSE);
          ucCntFlash = 0;
        }
        else
        {
          LedColor(LED_OFF);
        }
        break;
      case STATE_OFF:
        if (ucCntFlash++ > 4) 
        {
          LedColor(LED_RED);//RGB(TRUE,FALSE,FALSE);
          ucCntFlash = 0;
        }
        else
        {
//          LedColor(LED_OFF);//RGB(FALSE,FALSE,FALSE);
        }
        break;
      case STATE_HEATING:
        if (ucCntFlash++ > 3) 
        {
          LedColor(LED_GREEN);
          ucCntFlash = 0;
        }
        else
        {     
          LedColor(LED_RED);
        }
        break;
      case STATE_READY:
        if (ucCntFlash++ > 3) 
        {
          LedColor(LED_OFF);
          ucCntFlash = 0;
        }
        else
        {               
          LedColor(LED_GREEN);
        }
        break;
      case STATE_ON:
        LedColor(LED_GREEN);
        break;
      default:
//        gucDeviceState=0;
        break;
    }    
}

void Click(void)
{
	//Bei Einfachklick immer ausschalten
	gucDeviceState = STATE_OFF;
	gucDeviceStateShadow = gucDeviceState;

  GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_HI, LOW);   //Laser ausschalten
  GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_LO, LOW);   //Laser ausschalten
  GpioExpanderIc91.ModifyBuffer(IO_EN_HEATER_PWR, LOW);     //Heizwiderstand ausschalten
  GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, LOW);            //Peltier-Element ausschalten
  GpioExpanderIc91.SendBufferToI2c();
}

void DoubleClick(void)
{
  
}

void Click3sec()
{
  switch(gucDeviceState)
  {
    case STATE_OVERTEMP:
      gucDeviceState = STATE_HEATING;
      gucDeviceStateShadow = gucDeviceState;
      GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, HIGH);//Peltier-Element einschalten
      break;
    case STATE_OFF:
      gucDeviceState = STATE_HEATING;
      gucDeviceStateShadow = gucDeviceState;
      GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, HIGH);//Peltier-Element einschalten
      break;
    case STATE_HEATING:
      gucDeviceState = STATE_OFF;
      gucDeviceStateShadow = gucDeviceState;
      GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, LOW); //Peltier-Element ausschalten
      break;
    case STATE_READY:
      gucDeviceState = STATE_ON;
      gucDeviceStateShadow = gucDeviceState;
      GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_HI, HIGH);   //Laser einschalten
      GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_LO, HIGH);   //Laser einschalten
      GpioExpanderIc91.ModifyBuffer(IO_EN_HEATER_PWR, HIGH);     //Heizwiderstand einschalten
      break;
    case STATE_ON:
      gucDeviceState = STATE_OFF;
      gucDeviceStateShadow = gucDeviceState;
      GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_HI, LOW);   //Laser ausschalten
      GpioExpanderIc91.ModifyBuffer(IO_EN_LASER_PWR_LO, LOW);   //Laser ausschalten
      GpioExpanderIc91.ModifyBuffer(IO_EN_HEATER_PWR, LOW);     //Heizwiderstand ausschalten
      GpioExpanderIc91.ModifyBuffer(IO_EN_TEC, LOW);            //Peltier-Element ausschalten
      break;
    default:
      gucDeviceState = STATE_OFF;
      gucDeviceStateShadow = gucDeviceState;
      break;
  }
  GpioExpanderIc91.SendBufferToI2c();
}

void ReadSwitch()
{
#define SWITCH_OFF_DELAY 1000 //wie oft muss CycleTimer_Task auslösen, bis bei Tastendruck Geraet ausschaltet
#define CLICK_DELAY       3
#define DELAY_BETWEEN_DUBBLECLICK    5//50
	
#define NO_SWITCH_TO_HOLD 0
#define SWITCH_TO_HALT_PHASE_1  1
#define SWITCH_TO_HALT_PHASE_2  2
#define SWITCH_TO_HALT_PHASE_3  3
#define CLICK_PHASE_1           4
#define CLICK_PHASE_2           5

static unsigned char ucSwitchPhase = 0;
static unsigned int uiCntUntilSwitchOff = SWITCH_OFF_DELAY;
bool bSwitchState;

  //DelayMs(2); //Delay 2ms
  bSwitchState = digitalRead(GPIO_SWITCH);

  switch (ucSwitchPhase)
  {
    case NO_SWITCH_TO_HOLD:
      if(bSwitchState)//Taster gedrueckt=>POWER_SWITCH_PORT=0
      {//Taster nicht gedrueckt
        if(uiCntUntilSwitchOff < (SWITCH_OFF_DELAY - CLICK_DELAY))
        {//Taster wurde ueber einen Zeitraum von CLICK_DELAY gedrueckt
          ucSwitchPhase = CLICK_PHASE_1;
        }
        uiCntUntilSwitchOff = SWITCH_OFF_DELAY;//von vorn z�hlen
      }
      else
      {//Taster gedrueckt
        if((0==(uiCntUntilSwitchOff--)) && (0==ucSwitchPhase))//warte, bis Taster ueber 3 sec gedrueckt wurde
        {//Taster wurde jetzt ueber 3 sec gedrueckt
            ucSwitchPhase = SWITCH_TO_HALT_PHASE_1;
            Click3sec();
        }
      }
          break;
          
    case CLICK_PHASE_1:
      if(bSwitchState)//Taster gedrueckt=>POWERSWITCH_PORT=0
      {//Taster nicht gedrueckt
        if(uiCntUntilSwitchOff-- < (SWITCH_OFF_DELAY - DELAY_BETWEEN_DUBBLECLICK))
        {//Taster wurde lange nicht gedrueckt=> kein Doppelklick sondern Einfachklick
          Click(); //=> Click-Ereignis
          ucSwitchPhase = NO_SWITCH_TO_HOLD;
          uiCntUntilSwitchOff = SWITCH_OFF_DELAY;//von vorn zaehlen
        }
      }
      else
      {//Taster zum zweiten Mal gedrueckt, bevor DELAY_BETWEEN_DUBBLECLICK abgelaufen ist => DubbleClick
          DoubleClick(); //=> Doppel-Click-Ereignis
          ucSwitchPhase = CLICK_PHASE_2;
		  #ifdef DEBUG_MODUS	
		  _dbgwrite("Switch to : CLICK_PHASE_2\r\n");
		  #endif
          uiCntUntilSwitchOff = SWITCH_OFF_DELAY;//von vorn z?hlen
      }
            break;
            
    case CLICK_PHASE_2://warte nach Doppelklick, bis Taster losgelassen wurde
      if(bSwitchState)//Taster gedrueckt=>POWERSWITCH_PORT=0
      {//Taster nicht gedrueckt
          ucSwitchPhase = NO_SWITCH_TO_HOLD;
          uiCntUntilSwitchOff = SWITCH_OFF_DELAY;//von vorn zaehlen
      }
            break;
			
    case SWITCH_TO_HALT_PHASE_1://Taster wurde ueber 3 sec gedrueckt...
      if(bSwitchState)//...und jetzt losgelassen; warte noch eine Weile, bis Prellvorgaenge abgeschlossen sind
      {//Taster nicht gedrueckt
        ucSwitchPhase=SWITCH_TO_HALT_PHASE_2;
      }
           break;
          
    case SWITCH_TO_HALT_PHASE_2://Taster wurde schon ueber 3 sec gedrueckt und danach losgelassen
      //Taster wurde schon ueber 3 sec gedrueckt, danach losgelassen und Prellvorgaenge sind abgeschlossen => HALT
      ucSwitchPhase = NO_SWITCH_TO_HOLD;
      uiCntUntilSwitchOff = SWITCH_OFF_DELAY;//von vorn z?hlen
          break;
 
   default:
      ucSwitchPhase=NO_SWITCH_TO_HOLD;
          break;
  }
}