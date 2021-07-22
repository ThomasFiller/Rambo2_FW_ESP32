#ifndef __GlobalConsts_H_included
#define __GlobalConsts_H_included

#include <Arduino.h>
#include "stdint.h"
#include "globalconsts.h"

//#define PULSEMODE  //Anstiegszeit @ 400mA: Pulsmode ca. 100�s (komplett eingeschwungen nach ca. 400�s); ohne Pulsmode ca. 400�s (eingeschwungen)
//#define REGULATE_POWERSOURCE //Ausgeblendet, weil Regelung von gepulsten Lasern nicht funktioniert (Spannung am Laser wird nicht zuverl�ssig gemessen, wenn der gepulst betrieben wird!)

#define DEFAULT_UART_ADDRESS_RECEIVE    (uint8_t)0x2A
#define DEFAULT_UART_ADDRESS_TRANSMIT   (uint8_t)0x0A

#ifdef PULSEMODE
  #define REFREG_OFFSET 0xF0
#else
  #define REFREG_OFFSET 0x30
#endif

#define REG_0x1E_OFFSET 0x00

#define OVERTEMPERATURE 23400 //23400 entspricht etwa 45�C (siehe Tabelle "DAC_ADC_TECCTL" in "Q:\Organisation\intern\05_Elektronik\Messungen_Berechnungen.xlsx"

#define DCO_REG_THRES           76       
#define DCO_REG_HYSTERESIS      17

#define NO_OF_SETUPS 2
#define NO_OF_STORAGE_BYTES 25
#define NO_OF_LASERS 12
#define NO_OF_300mALASERS 2
#define NO_OF_HEATERS 4
#define NO_OF_DACS 4

#define INDEX_MAX_TEC_HEAT_CURRENT 6

#define BIT_TIA_AVAILABLE 0
#define BIT_LIA_DIGITAL_AVAILABLE 1

#define BIT_IC91_AVAILABLE 3
#define BIT_IC94_AVAILABLE 4

//*****************************States des Zustandsautomaten zum Abfragen der ADCs im iC-HTP und HumSensor
#define I_LAS                           0 //Strom durch die einzelnen Laser
#define MDK                             1 //FlussSpannungen der Laser an den 300mA-Treiern lesen  
#define VB_VDD                          2 //Versorgungsspannungen Digitalelektronik iC-HTP
#define VBL                             3 //Versorgungsspannungen Lasertreiber
#define LDA                             4 //Flussspannungen an den einzelnen Lasern
#define BOARD_TEMPERATURE_AND_HUMIDITY  5
#define EVALUATION_SOURCEVOLTAGE        6
#define TEC                             7 //Temperatur und Spannung am Peltier-Element mit ADC Ads1015 lesen 
//#define LIA                             8 //LockInAmp Spannung lesen

//******************************I2C-Adressen
/*UcEspInterBoard.SchDoc 1 für LIA
         ICi1   IO-Expander PCF8575     SCL1    0100 010X =  0x44 (0100XXX, S. 16)    Pin18(A0)->GND; Pin23(A1)->3V3; Pin24(A2)->GND
         ICi2   ADC ADS1015IDGSR        SCL1    1001 001X =  0x92 (10010XX, S.19)      Pin1 -> +5V5
UcEspInterBoard.SchDoc 2 für TIA
         ICi1   IO-Expander PCF8575     SCL1    0100 111X =  0x4E (0100XXX, S. 16)    Pin18(A0)->3V3; Pin23(A1)->3V3; Pin24(A2)->3V3
         ICi2   ADC ADS1015IDGSR        nicht bestückt

Display
         SSD1306                        SCL1    0011 110X =  0x3C
uC.SchDoc
         IC30   Feuchtesensor HDC2080   SCL1    1000 000X =  0x80 (100000X, S.15)     Pin3(ADDR) -> GND
         IC40   DAC   LTC2635           SCL1    1110 010X =  0xE4 (XXX00XX, S.22)     Pins 9,7,4 -> +5V
UcEspEnLas.SchDoc
         IC91   IO-Expander PCF8575     SCL1    0100 000X =  0x40 (0100XXX, S. 16)    Pin18(A0)->GND; Pin23(A1)->GND; Pin24(A2)->GND
         IC94   IO-Expander PCF8575     SCL1    0100 001X =  0x42 (0100XXX, S. 16)    Pin18(A0)->3V3; Pin23(A1)->GND; Pin24(A2)->GND
HeaterDriver.SchDoc MEHRERE!!!!!!!!!!!!!!!!!!!!!!!!
         IC7    iC-HTP                  SCL3    1000 0XXX =  0x8X (10000XX, S.28)
ADC.SchDoc
         IC080  ADC ADS7828             SCL2    1001 011X =  0x96 (10010XX, S.11)
LaserDriver.SchDoc                                                                       R1             R2
      A  IC1    iC-HTP                  SCL2    1000 001X =  0x82 (10000XX, S.28)      Pin12(A1)->GND; Pin11(A0)->3V3
      B  IC1    iC-HTP                  SCL2    1000 011X =  0x86 (10000XX, S.28)      Pin12(A1)->3V3; Pin11(A0)->3V3
      C  IC1    iC-HTP                  SCL1    1000 011X =  0x86 (10000XX, S.28)      Pin12(A1)->3V3; Pin11(A0)->3V3
      D  IC1    iC-HTP                  SCL1    1000 010X =  0x84 (10000XX, S.28)      Pin12(A1)->3V3; Pin11(A0)->GND
      E  IC1    iC-HTP                  SCL1    1000 001X =  0x82 (10000XX, S.28)      Pin12(A1)->GND; Pin11(A0)->3V3
TEC-CTL.SchDoc
         IC2    ADC ADS1015IDGSR        SCL2    1001 000X =  0x90 (100100X, S.19)      Pin1 -> GND
         IC3    DAC LTC2635             SCL2    1110 010X =  0xE4 (XXX00XX, S.22)      Pins 9,7,4 -> +5V5
TEC-CTL.SchDoc (zweiter TEC-Controller für SHG)
         IC2    ADC ADS1015IDGSR        SCL2    1001 001X =  0x92 (10010XX, S.19)      Pin1 -> +5V5
         IC3    DAC LTC2635             SCL2    0010 000X =  0x20 (XXX00XX, S.22)      Pins 9,7,4 -> GND*/

#define I2C_ADDRESS_TEMP_HUM_SENSOR     0x40
#define I2C_ADDRESS_DAC1                0x72
#define I2C_ADDRESS_DAC2                0x10
#define I2C_ADDRESS_ADC1                0x48//1001000
#define I2C_ADDRESS_ADC2                0x49//1001001
#define I2C_ADDRESS_LAS_DRV_A           0x41       
#define I2C_ADDRESS_LAS_DRV_B           0x43
#define I2C_ADDRESS_LAS_DRV_C           0x43
#define I2C_ADDRESS_LAS_DRV_D           0x42
#define I2C_ADDRESS_LAS_DRV_E           0x41
#define I2C_ADDRESS_HEAT_DRV_0          0x40
#define I2C_ADDRESS_HEAT_DRV_1          0x41
#define I2C_ADDRESS_HEAT_DRV_2          0x42
#define I2C_ADDRESS_HEAT_DRV_3          0x43

#define I2C_ADDRESS_GPIO_EXPANDER_UCBOARD_IC94 0x21
#define I2C_ADDRESS_GPIO_EXPANDER_UCBOARD_IC91 0x20

#define I2C_ADDRESS_GPIO_EXPANDER_Tia 0x27
#define I2C_ADDRESS_GPIO_EXPANDER_Lia 0x22
#define I2C_ADDRESS_ADC_Lia 0x49

//*****************************I2C-Kan�le
#define SCL1 1
#define SCL2 2
#define SCL3 3

#define I2C_CHAN_LAS_DRV_A      SCL2       
#define I2C_CHAN_LAS_DRV_B      SCL2
#define I2C_CHAN_LAS_DRV_C      SCL1
#define I2C_CHAN_LAS_DRV_D      SCL1
#define I2C_CHAN_LAS_DRV_E      SCL1
#define I2C_CHAN_HEAT_DRV_0     SCL3

#define I2C_CHAN_DAC            SCL2
#define I2C_CHAN_ADC            SCL2

//******************************SPI-Adressen
#define SPI_ADDRESS_LAS_DRV_10     0x1//0x0
#define SPI_ADDRESS_LAS_DRV_11     0x0//0x1

//******************************Device states
#define STATE_OFF 0
#define STATE_HEATING 1
#define STATE_READY 2
#define STATE_ON 3
#define STATE_OVERTEMP 4
#define STATE_REGULATION_IN_PROGRESS 5

#define DELAY_AFTER_READY_TEC_VOLTAGE 100

//*******************************UART-Commands
//Allgemein; int. Trigger; Setup
#define	UART_ERROR           	0x00	//	0	1	Errorcode (see below)
#define	UART_INTTRIG_FREQ	0x01	//	2	2	Frequenz mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet; bei Data=0 wird interner Trigger abgeschaltet
#define	UART_INTTRIG_RAT	0x02	//	2	2	Tastverh�ltnis mit der der interne Trigger zwischen den beiden Wellenl�ngen umschaltet
#define	UART_TRIG_FUNCTION	0x07	//	1	1	Triggerfunktion: 0-interner Trigger, 1-extern rising edge, 2-extern falling edge, 3-extern state	

#define	UART_SETUP_NO	        0x03	//	1	1	liest oder schreibt eine Setupnummern, die sich parrallel auch mit dem Schalter einstellen und �ber die LEDs ablesen lassen
#define	UART_SAVE_SETUP	        0x04	//	1	1	veranlasst den uC, das aktuelle Setup und die aktuelle Setupnummer zu speichern
#define UART_DEVICE_STATE       0x05    //      /       1       liest den aktuellen Status des RDD
#define	UART_STORAGE	        0x08	//	2	2	16 Reservedaten zum Abspeichern (z.Bsp. Tmax, Tmin�); 1. Byte: Adresse (0..15); zweites Byte: Datum
    #define T_MIN           0
    #define T_MAX           1
    #define I_MAX_LAS_0     2
    #define I_MAX_LAS_1     3
    #define I_MAX_LAS_2     4
    #define I_MAX_LAS_3     5
    #define I_MAX_LAS_4     6
    #define I_MAX_LAS_5     7
    #define I_MAX_LAS_6     8
    #define I_MAX_LAS_7     9
    #define I_MAX_LAS_8     10
    #define I_MAX_LAS_9     11
    #define I_MAX_HEATER0   12
    #define I_MAX_HEATER1   13
    #define I_MAX_HEATER2   14
    #define I_MAX_HEATER3   15
    #define SER_NO          16
    #define NO_OF_GROUPS    17
    #define FIRST_DRV_1     18
    #define LAST_DRV_1      19
    #define FIRST_DRV_2     20
    #define LAST_DRV_2      21


//Umweltdaten uC-Board						
#define	UART_BOARD_TEMPERATURE	0x09	//	/	2	Temperatur auf uC-Board
#define	UART_BOARD_HUMIDITY	0x0A	//	/	2	Luftfeuchte auf uC-Board

#define	UART_READ_ALL	        0x0B	//	/	?	Liest alle Daten, die f�r zyklische Anzeige ben�tigt werden		
#define	UART_ICHTP_NO_TO_READ	0x0C	//	/	31	Legt fest, von welchem iC-HTTP alle Daten gelesen werden sollen
#define	UART_READ_ICHTP	        0x0D	//	/	31	Liest alle iC-HTP-Register des mit UART_ICHTP_NO_TO_READ gew�hlten Typs
#define	UART_READ_ADC_LASER     0x0E    //	/	50	Liest alle Analogwerte, die mit iC-HTPs gewandelt wurden											
#define	UART_READ_ADC_HEATER	0x0F	//	/	20	Liest alle Analogwerte, die mit iC-HTPs gewandelt wurden	

//Lasertreiber und Teiber f�r Heizwiderstand			//			
#define	UART_EN_LAS_HEAT_PWR    0x06	//	1	1	Einschalten des Spannungsreglers f�r die Versorgung der Laserstromquellen und des Stromes durch den Heizwiderstand (0-aus; 1-an)
#define	UART_DCO_HI	        0x20	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Laserstromquellen LAS_PWR_HI
#define	UART_DCO_LO	        0x21	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Laserstromquellen LAS_PWR_LOW
#define	UART_DCO_RES0	        0x22	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Stromquelle f�r den Heizwiderstand 0
#define	UART_DCO_RES1	        0x23	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Stromquelle f�r den Heizwiderstand 1
#define	UART_DCO_RES2	        0x24	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Stromquelle f�r den Heizwiderstand 2
#define	UART_DCO_RES3	        0x25	//	1	1	Strom zur Regulierung der Spannungsregler f�r die Versorgung der Stromquelle f�r den Heizwiderstand 3
#define	UART_MAX_TEC_HEAT_CURRENT	0x26	//	1	1	Strom der in Spannung gewandelt wird, um Maximalen Heizstrom am TEC-Treiber einzustellen

#define	UART_AVAILABLE_ICHTP	0x2F	//	/	2	Bitmuster der zur Verf�gung stehenden iC-HTPs

#define	UART_I_LAS0	        0xA0	//	2	2	Einstellen des Laserstromes Treiber 0
#define	UART_I_LAS1	        0xA1	//	2	2	Einstellen des Laserstromes Treiber 1
#define	UART_I_LAS2	        0xA2	//	2	2	Einstellen des Laserstromes Treiber 2
#define	UART_I_LAS3	        0xA3	//	2	2	Einstellen des Laserstromes Treiber 3
#define	UART_I_LAS4	        0xA4	//	2	2	Einstellen des Laserstromes Treiber 4
#define	UART_I_LAS5	        0xA5	//	2	2	Einstellen des Laserstromes Treiber 5
#define	UART_I_LAS6	        0xA6	//	2	2	Einstellen des Laserstromes Treiber 6
#define	UART_I_LAS7	        0xA7	//	2	2	Einstellen des Laserstromes Treiber 7
#define	UART_I_LAS8	        0xA8	//	2	2	Einstellen des Laserstromes Treiber 8
#define	UART_I_LAS9	        0xA9	//	2	2	Einstellen des Laserstromes Treiber 9

#define	UART_I_HEATER0	        0xAA	//	2	2	Strom am Heizwiderstand einstellen
#define	UART_I_HEATER1	        0xAB	//	2	2	Strom am Heizwiderstand einstellen
#define	UART_I_HEATER2	        0xAC	//	2	2	Strom am Heizwiderstand einstellen
#define	UART_I_HEATER3	        0xAD	//	2	2	Strom am Heizwiderstand einstellen

#define	UART_I_LAS10	        0xAE	//	2	2	Einstellen des Laserstromes Treiber 10 (300mA-Treiber f�r schnelle Schaltvorg�nge)
#define	UART_I_LAS11	        0xAF	//	2	2	Einstellen des Laserstromes Treiber 11 (300mA-Treiber f�r schnelle Schaltvorg�nge)

#define	UART_EN_LAS0	        0xB0	//	1	1	Wann ist Laser0 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS1	        0xB1	//	1	1	Wann ist Laser1 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS2	        0xB2	//	1	1	Wann ist Laser2 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS3	        0xB3	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS4	        0xB4	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS5	        0xB5	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS6	        0xB6	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS7	        0xB7	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS8	        0xB8	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
#define	UART_EN_LAS9	        0xB9	//	1	1	Wann ist Laser3 an: 0-immer aus; 1-bei Triggersignal an; 2-bei invertiertem Triggersignal an; FF-immer an
			
//TEC-Control			//			
#define	UART_EN_TEC	        0x30	//	1	1	Peltier-Element einschalten
#define	UART_VTEC	          0x32	//	0	2	liest den aktuellen Strom durch das Peltierelement
#define	UART_ITEC	          0x33	//	0	2	liest die aktuelle Spannung am Peltierelement
#define	UART_TMP	          0x34	//	0	2	liest die aktuelle Temperatur
#define	UART_TMP_SET	      0x35	//	2	2	Solltemperatur einstellen

//Freie DAC-Kan�le
#define	UART_DAC_B	        0x36	//	2	2
#define	UART_DAC_C	        0x37	//	2	2
#define	UART_DAC_D	        0x38	//	2	2

//Lock-In-Amplifier						
#define	UART_LIA_DIGITAL	  0x41	//	2	3	Schreiben: Byte1->Pin10..17; Byte2->Pin18..25 |  Lesen Byte1,2 wie Schreiben, Byte3->Pin6..7
#define	UART_LIA_SIGNAL	    0x42	//	0	2	Messwert des ADC auf dem InterBoard
#define	UART_LIA_MEASURING_RANGE	0x43	//	1	1	Messbereich 0000 : FSR = ±6.144 V; 0010 : FSR = ±4.096 V; 0100 : FSR = ±2.048 V; 0110 : FSR = ±1.024 V; 1000 : FSR = ±0.512 V; 1010 : FSR = ±0.256 V; 1100 : FSR = ±0.256 V; 1110 : FSR = ±0.256 V
#define	UART_LIA_FILTER_DEPTH	0x44	//	1	1	

//TransImpedance-Amplifier						
#define	UART_TIA	          0x40	//	2 2 	bit 0..7 -> Pin5, Pin10, Pin11, Pin12, Pin13, Pin14; bit14, bit15 -> LIA-Pins 6 und 7


//*******************************ERROR-Codes
#define	NON_ERROR			0
#define	ERROR_WRONG_CMD			4
#define	ERROR_CHECKSUM			7

/////////////////////////////////////////////////////////------------------GPIO--------------------PORTS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define GPIO_TEST_PIN 0

//*******************************I2C-Port
#define GPIO_I2C_SDA        21
#define GPIO_I2C_SCL        22
#define GPIO_SELECT_I2C     33
#define GPIO_SELECT_I2C_2   25
//SELECT_I2C | SELECT_I2C_2 | Activ SCL
//-----------|--------------|-----------
//    0      |     X        | I2C_SCL_1
//    1      |     0        | I2C_SCL_2
//    1      |     1        | I2C_SCL_3

//******************************SPI-Ports
#define GPIO_SPI_nCS     4
#define GPIO_SPI_SCK     17
#define GPIO_SPI_MOSI    16
#define GPIO_SPI_MISO    32

#define GPIO_SPI_ADC_nCS     5
#define GPIO_SPI_ADC_SCK     18
#define GPIO_SPI_ADC_MOSI    23
#define GPIO_SPI_ADC_MISO    19

#define GPIO_SPI_DAC_nCS     13//34 geht nicht, da Input only
#define GPIO_SPI_DAC_SCK     26
#define GPIO_SPI_DAC_MOSI    27

//******************************UART-Port
#define GPIO_USB_UART_RXD 1
#define GPIO_USB_UART_TXD 3

//******************************TEC-Controller Port
#define IO_EN_TEC P6

//******************************Setup-Taster
#define GPIO_SWITCH 36

//******************************Trigger-Ausgang synchron zu internem Generator bzw. -Eingang zur externen Triggerung
#define GPIO_TRIGGER_OUT      15//0//35 geht nicht, da Input only //gleicher Pin wie GPIO_PULSE
#define GPIO_TRIGGER_IN       39
#define IO_TRIGGER_nPULLUP  P15

//******************************Spannungsversorgung Heizwiderstand
#define IO_EN_HEATER_PWR P7

//******************************Versorgung und Ansteuerung Laser-Driver
#define IO_EN_LASER_PWR_HI P5//Spannungsregler zur Versorgung der Lasertreiber
#define IO_EN_LASER_PWR_LO P4

  //IC94
#define IO_PULL_EN_LAS1  P8//Pull up oder pull down der Laserausgänge
#define IO_PULL_EN_LAS0  P9
#define IO_PULL_EN_LAS3  P10
#define IO_PULL_EN_LAS2  P11
#define IO_PULL_EN_LAS5  P12
#define IO_PULL_EN_LAS4  P13
#define IO_PULL_EN_LAS7  P14
#define IO_PULL_EN_LAS6  P15
  //IC91
#define IO_PULL_EN_LAS9  P11
#define IO_PULL_EN_LAS8  P10
#define IO_PULL_EN_LAS11  P9
#define IO_PULL_EN_LAS10  P8

  //IC94
#define IO_PRG0_LAS1  P0
#define IO_PRG0_LAS0  P1
#define IO_PRG0_LAS3  P2
#define IO_PRG0_LAS2  P3
#define IO_PRG0_LAS5  P4
#define IO_PRG0_LAS4  P5
#define IO_PRG0_LAS7  P6
#define IO_PRG0_LAS6  P7
  //IC91
#define IO_PRG0_LAS9   P3
#define IO_PRG0_LAS8   P2
#define IO_PRG0_LAS11  P1
#define IO_PRG0_LAS10  P0

#define IC94 0
#define IC91 1

  //IC94
#define IC_LAS0  IC94 //GPIO-Expander-IC, über welchen der Enable-Pin des Lasers programmiert wird
#define IC_LAS1  IC94 //GPIO-Expander-IC, über welchen der Enable-Pin des Lasers programmiert wird
#define IC_LAS2  IC94
#define IC_LAS3  IC94
#define IC_LAS4  IC94
#define IC_LAS5  IC94
#define IC_LAS6  IC94
#define IC_LAS7  IC94
  //IC91
#define IC_LAS8   IC91
#define IC_LAS9   IC91
#define IC_LAS10  IC91
#define IC_LAS11  IC91

//******************************Display-LED
#define IO_LED1_PLUS  P12 //LEDrd
#define IO_LED1_MINUS P13 //LEDgn
#define IO_LED1_BU    P14 //LEDgn

/*-------------------------------------------------------------------------
 *      Interrupt vector numbers
 *-----------------------------------------------------------------------*/
#define TRAP_vector                          0x01
//RESERVED
#define FLASH_vector                         0x03
#define DMA1_CH0_CH1_vector                  0x04
#define DMA1_CH2_CH3_vector                  0x05
#define RTC_vector                           0x06
#define EXTIE_PVD_vector                     0x07
#define EXTIB_vector                         0x08
#define EXTID_vector                         0x09
#define EXTI0_vector                         0x0A
#define EXTI1_vector                         0x0B
#define EXTI2_vector                         0x0C
#define EXTI3_vector                         0x0D
#define EXTI4_vector                         0x0E
#define EXTI5_vector                         0x0F
#define EXTI6_vector                         0x10
#define EXTI7_vector                         0x11
//LCD
#define CLK_CSS_vector                       0x13
#define CLK_SWITCH_vector                    0x13
#define TIM1_BIF_vector                      0x13
#define COMP_EF1_EF2_vector                  0x14
#define TIM2_OVR_UIF_vector                  0x15
#define TIM2_CAPCOM_TIF_vector               0x16
#define TIM3_OVR_UIF_vector                  0x17
#define TIM3_CAPCOM_TIF_vector               0x18
#define TIM1_OVR_UIF_vector                  0x19
#define TIM1_CAPCOM_vector                   0x1A
#define TIM4_TIF_UIF_vector                  0x1B
#define SPI_vector                           0x1C
#define USART_T_vector                       0x1D
#define USART_R_vector                       0x1E
#define I2C_vector                           0x1F

#define USART_StopBits_1   (uint8_t)0x00
#define USART_Mode_Tx     (uint8_t)0x08
#define USART_Mode_Rx     (uint8_t)0x04


//######################################################################################|
//                                                                                      |
//      Bit manipulation macros                                                         |
//                                                                                      |
//######## begin      ##################################################################|

#ifndef __MACROS_H_included
#define __MACROS_H_included

#define __BIT_MASK(A) (1<<(A))
#define SetBitMask( SFR, MASK ) 		((SFR)	|=	(MASK))
#define ClrBitMASK( SFR, MASK ) 		((SFR)	&=	~(MASK))
#define CheckBit1( X, MASK ) 		((X) 		& 	(MASK))
#define ToggleBit( SFR, MASK )	((SFR)	^=	(MASK))


#define SetBit(VAR,Place)         ( (VAR) |= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define ClrBit(VAR,Place)         ( (VAR) &= (uint8_t)((uint8_t)((uint8_t)1<<(uint8_t)(Place))^(uint8_t)255) )

#define ChgBit(VAR,Place)         ( (VAR) ^= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define AffBit(VAR,Place,Value)   ((Value) ? \
                                   ((VAR) |= ((uint8_t)1<<(Place))) : \
                                   ((VAR) &= (((uint8_t)1<<(Place))^(uint8_t)255)))
#define MskBit(Dest,Msk,Src)      ( (Dest) = ((Msk) & (Src)) | ((~(Msk)) & (Dest)) )

#define ValBit(VAR,Place)         ((uint8_t)(VAR) & (uint8_t)((uint8_t)1<<(uint8_t)(Place)))

#define DEBUGGING             // Auskommentieren wenn keine Serielle Ausgabe erforderlich ist

#ifdef DEBUGGING
#define DEBUG(...) Serial1.println(__VA_ARGS__)
#define DEBUG_F(...) Serial1.printf("Funktion: %s meldet in Zeile: %d -> ", __PRETTY_FUNCTION__, __LINE__); Serial1.println(__VA_ARGS__)
#else
#define DEBUG(...)
#define DEBUG_F(...)
#endif

//########   end    Bit manipulation macros       ######################################|
#endif

//***************type definitions for a combined 8-bit/16-bit and byte/word access*******************
//used for 16-bit calculations and 8-bit register loading
typedef struct
{
	uint8_t	ucLowByte;
	uint8_t	ucHighByte;
} typDoubleByte;

typedef union
{
	uint16_t	uiWord;
	typDoubleByte	stBytes;
} typUnsignedWord;

typedef union
{
	int16_t	iWord;
	typDoubleByte	stBytes;
} typSignedWord;

typedef struct
{
	uint8_t	ucLowByte;
	uint8_t	ucMedLowByte;
	uint8_t	ucMedHighByte;
	uint8_t	ucHighByte;
} typQuadByte;

typedef union
{
	int32_t	i32Word;
	typQuadByte	stBytes;
} typSigned32;

//****************************************************************************************************

#endif