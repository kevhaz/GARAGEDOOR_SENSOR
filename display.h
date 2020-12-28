#pragma once

#include <SSD1306.h>

// OLED pins to ESP32 GPIOs via these pins:
#define OLED_ADDR 0x3c
#define OLED_SDA  4 //  GPIO4
#define OLED_SCL  15 // GPIO15
#define OLED_RST  16 // GPIO16

SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL);


// ----------------------------------------------------------------  
// Initilize the OLED functions.
//
// ----------------------------------------------------------------
void init_Display() 
{
#if defined OLED_RST
  pinMode(OLED_RST,OUTPUT);
  digitalWrite(OLED_RST, LOW);  // low to reset OLED
  delay(50); 
  digitalWrite(OLED_RST, HIGH);   // must be high to turn on OLED
  delay(50);
#endif  
  // Initialising the UI will init the display too.
  Serial.println("init_Display called");
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 24, "STARTING");
  display.display();
  Serial.println("init_Display complete");
  return;
}

// ----------------------------------------------------------------
// Display up to 4 lines of text.
//
// ----------------------------------------------------------------
void display_Lines(String msg, String scndLine = "", String thrdLine = "" , String fourthLine = "") 
{
  // Initialising the UI will init the display too.
  //Serial.println( "display_Lines called");
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setColor(WHITE); // the only colour us black and white
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0,  0, msg.c_str());
  display.setFont(ArialMT_Plain_16);  
  display.drawString(0, 16, scndLine.c_str()); 
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 54, thrdLine.c_str());
  display.drawString(0, 44, fourthLine.c_str());
  display.display();
  //Serial.println( "display_Lines complete");
  return;
}
