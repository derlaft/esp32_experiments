#include <Arduino.h>

#include <GxEPD.h>

// select the display class to use, only one, copy from GxEPD_Example
// #include <GxGDEP015OC1/GxGDEP015OC1.h>
#include <GxGDEH0154D67/GxGDEH0154D67.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

#include <Adafruit_I2CDevice.h>

// constructor for AVR Arduino, copy from GxEPD_Example else
GxIO_Class io(SPI, /*CS*/ 26, /*DC=*/ 25, /*RST=*/ 18);
GxEPD_Class display(io, /*RST=*/ 33, /*BUSY=*/ 27); 

void loop() {};

void drawHelloWorld()
{
  display.setTextColor(GxEPD_BLACK);
  display.print("Hello World!");
}


void setup()
{
  display.init();
  display.eraseDisplay();
  // comment out next line to have no or minimal Adafruit_GFX code
  display.drawPaged(drawHelloWorld); // version for AVR using paged drawing, works also on other processors
}

