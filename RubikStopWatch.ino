/*
 * based on https://github.com/DeanIsMe/SevSeg/blob/master/examples/testWholeDisplay/testWholeDisplay.ino
 */

#include "Arduino.h"


#include <SevSeg.h>

SevSeg sevseg; //Instantiate a seven segment controller object
#define WAIT_MS 1000
unsigned long start_time_ms = 0;
long num = 0;

void setup()
{
  byte numDigits = 4;
  byte digitPins[] = {2, 3, 4, 5}; //Digits: 1,2,3,4 <--put one resistor (ex: 220 Ohms, or 330 Ohms, etc, on each digit pin)
  byte segmentPins[] = {13, 12, 11, 10, 9, 8, 7, 6}; //Segments: A,B,C,D,E,F,G,Period
  byte resistorsOnSegm = 1;

  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins, resistorsOnSegm);
  sevseg.setBrightness(10); //Note: 100 brightness simply corresponds to a delay of 2000us after lighting each segment. A brightness of 0
                            //is a delay of 1us; it doesn't really affect brightness as much as it affects update rate (frequency).
                            //Therefore, for a 4-digit 7-segment + pd, COMMON_ANODE display, the max update rate for a "brightness" of 100 is 1/(2000us*8) = 62.5Hz.
                            //I am choosing a "brightness" of 10 because it increases the max update rate to approx. 1/(200us*8) = 625Hz.
                            //This is preferable, as it decreases aliasing when recording the display with a video camera....I think.
  sevseg.setNumber(num, 0);
  start_time_ms = millis();
}

void loop()
{
  //local vars
  static byte decPlace = 0;
  
  if (millis() - start_time_ms >= 1000)
  {
      num < 9999 ? num++ : num = 0;
      sevseg.setNumber(num, decPlace);
      start_time_ms = millis();
      //decPlace++;
      //decPlace %= 2; //rollover back to 0 once variable gets to 2
  }



  sevseg.refreshDisplay(); // Must run repeatedly; don't use blocking code (ex: delay()) in the loop() function or this won't work right
}
