#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"
SSD1306Wire display(0x3c, SDA, SCL);   

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  60

void setup() {

  // Serial.begin(115200);

  // Initialising the UI will init the display too.
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // TODO: test longevity in deep sleep mode
  {
    display.displayOn();
    display.clear();
    display.drawString(0, 0, "Hello world");
    display.drawProgressBar(0, 32, 120, 10, 50);
    display.display();
    delay(500);
    display.displayOff();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

}

int counter = 0;

void loop() {

  counter++;

  // clear the display
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Hello world");

  int progress = (counter / 5) % 100;
  display.drawProgressBar(0, 32, 120, 10, progress);

  display.display();

  if (counter % 500 == 0) {
    // display.displayOff();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

  delay(10);
}
