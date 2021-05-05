
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Display.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void DisplayShow(uint8_t u8State)
{/*
char strDisplay[30];
//  DEBUG("DisplayShow(" + (String)u8State + ")");
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner

  sprintf(strDisplay, "UVC Flood Lamp No%d",gu16SerialNumber.uiWord);
  display.println(strDisplay);

  display.print(F("Address "));
  display.println((gIP));
  display.print(F("tot "));
  sprintf(strDisplay, "%d",gu32CntMinutes.ui32Word);
  display.print(strDisplay);
  display.println(" min");
 
  sprintf(strDisplay, "%.1f",gfConTemperature);
  display.print(strDisplay);
  display.write(248);  //°-Zeichen
  display.print(F("C RelHum="));
  sprintf(strDisplay, "%.0f",gfConHumidity);
  display.print(strDisplay);
  display.println(F("%"));
  
  display.println(F(""));

  if(OPERATION_NORMAL == gu8Operation)
  {
    display.print(F("Dimmer: "));
    display.print(gu8DimmerTarget);
    display.println(F("/63"));

    switch(u8State)
    {
      case STATE_OFF:
        display.print(F("Short Hit: "));
        display.print(gu16TimerSet.uiWord);
        display.println(F("s"));
        display.println(F("Long Hit: Switch On"));
  
        digitalWrite(LED_BU_PIN, LOW);
        digitalWrite(LED_GN_PIN, LOW);
        digitalWrite(LED_RD_PIN, HIGH);
        break;            
      case STATE_TIMER:
        display.println("Timer runnung " + (String)(gu16TimerSet.uiWord-gu16Timer));
        display.println(F("Hit: Switch Off"));
          
        digitalWrite(LED_BU_PIN, HIGH);
        digitalWrite(LED_GN_PIN, LOW);
        digitalWrite(LED_RD_PIN, LOW);
        break;
      case STATE_ON:
        display.println("On for " + (String)gu16Timer + "s");
        display.println(F("Hit: Switch Off"));
  
        digitalWrite(LED_BU_PIN, LOW);
        digitalWrite(LED_GN_PIN, HIGH);
        digitalWrite(LED_RD_PIN, LOW);   
        break;
      default:
        break;
    }  
  }
  else
  {
    digitalWrite(LED_BU_PIN, LOW);
    digitalWrite(LED_GN_PIN, LOW);
    digitalWrite(LED_RD_PIN, HIGH);
    display.println();
    display.println(strDisplayOperation[gu8Operation]);   
  }

  display.display();
//DEBUG("Write to Display");
*/
}

void DisplaySetup() 
{
  /*
char strDisplay[30];
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    DEBUG("SSD1306 allocation failed");
    //for(;;); // Don't proceed, loop forever
  }
DEBUG("SSD1306 allocated");
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
//  delay(200); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  display.setTextSize(1);      // Normal 1:1 pixel scale
//  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE); // Draw white text
//  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  sprintf(strDisplay, "UVC Flood Lamp No%d",gu16SerialNumber.uiWord);
  display.println(strDisplay);
  display.display();
  */
}
