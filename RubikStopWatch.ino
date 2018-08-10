/*
 * based on https://github.com/DeanIsMe/SevSeg/blob/master/examples/testWholeDisplay/testWholeDisplay.ino
 */

#include "Arduino.h"
#include <EEPROM.h>
#include <SevSeg.h>

#define BUTTON_DEBOUNCE_TIME_MS 50
#define CLR_WAIT_TIME_MS 3000u
#define RECORD_MAX 99999UL
#define E2_START_ADDR 14U

SevSeg sevseg; //Instantiate a seven segment controller object

unsigned long start_time_ms = 0;
unsigned long show_result_time_ms = 0;
unsigned long button_debounce_time_ms = 0;
unsigned long clr_wait_time_ms = 0;
unsigned long blink_time_ms = 0;
unsigned long num = 0;
bool is_new_record = false;
bool is_displ_blank = false;
int startBtn1Pin = A0;
int startBtn1state = HIGH;
int startBtn2Pin = A1;
int startBtn2state = HIGH;
int clrBtnPin = A2;
int clrBtnState = HIGH;

unsigned long e2_rec;

const char allDashesStr[] = "----";

typedef enum
{
	STATE_WAIT_FOR_BTN_DOWN = 0,
	STATE_CLR_RECORD,
	STATE_WAIT_FOR_BTN_UP,
	STATE_COUNTING,
	STATE_SHOW_RESULT
} StopWatchState_t;

StopWatchState_t StopWatchState = STATE_WAIT_FOR_BTN_DOWN;


void dispaly_counts(uint32_t rec);

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
	pinMode(startBtn1Pin, INPUT);
	pinMode(startBtn2Pin, INPUT);
	EEPROM.get(E2_START_ADDR, e2_rec);

	dispaly_counts(e2_rec);

}

void loop()
{
	switch (StopWatchState)
	{
	case STATE_WAIT_FOR_BTN_DOWN:
	{
		/* check if we need to reset counting */
		startBtn1state = digitalRead(startBtn1Pin);
		startBtn2state = digitalRead(startBtn2Pin);
		clrBtnState = digitalRead(clrBtnPin);
		if (startBtn1state == LOW && startBtn2state == LOW)
		{
			/* TODO: add delay in for initial setting value 0 on display
			 * to force user to keep button pressed for more than 1 loop() execution
			 */
			/* debounce LOW state of button */
			if (0 == button_debounce_time_ms)
			{
				button_debounce_time_ms = millis();
			}

			if (millis() - button_debounce_time_ms >= BUTTON_DEBOUNCE_TIME_MS)
			{
				num = 0;
				dispaly_counts(num);
				StopWatchState = STATE_WAIT_FOR_BTN_UP;
			}
		}
		else if (clrBtnState == LOW)
		{
			clr_wait_time_ms = millis();
			StopWatchState = STATE_CLR_RECORD;
		}
		else
		{
			/* do nothing */
		}
		break;
	}
	case STATE_CLR_RECORD:
	{
		clrBtnState = digitalRead(clrBtnPin);
		if (clrBtnState == LOW)
		{
			if (millis() - clr_wait_time_ms >= CLR_WAIT_TIME_MS)
			{
				/* clear eeprom */
				e2_rec = 0xFFFFFFFFUL;
				EEPROM.put(E2_START_ADDR, e2_rec);
				sevseg.setChars(allDashesStr);
			}
		}
		else
		{
			/* reset counting */
			StopWatchState = STATE_WAIT_FOR_BTN_DOWN;
		}
	}
	break;
	case STATE_WAIT_FOR_BTN_UP:
	{
		startBtn1state = digitalRead(startBtn1Pin);
		startBtn2state = digitalRead(startBtn2Pin);
		if (startBtn1state == HIGH && startBtn2state == HIGH)
		{
			start_time_ms = micros();
			StopWatchState = STATE_COUNTING;
		}
		break;
	}
	case STATE_COUNTING:
	{
		if (micros() - start_time_ms >= 1000)
		{
			num < RECORD_MAX ? num++ : num = 0;
			dispaly_counts(num);
			start_time_ms = micros();
		}

		startBtn1state = digitalRead(startBtn1Pin);
		startBtn2state = digitalRead(startBtn2Pin);
		if (startBtn1state == LOW && startBtn2state == LOW)
		{
			/* stop counting */
			show_result_time_ms = millis();
			blink_time_ms = show_result_time_ms;
			StopWatchState = STATE_SHOW_RESULT;
		}
		break;
	}
	case STATE_SHOW_RESULT:
	{
		if(num < e2_rec)
		{
			/* new record */
			e2_rec = num;
			EEPROM.put(E2_START_ADDR, e2_rec);
			is_new_record = true;
		}

		/* last counter value of num shall be already displayed
		 * keep it for 5 secs allowing user to release button */
		if (millis() - show_result_time_ms >= 5000)
		{
			/* reset state machine */
			StopWatchState = STATE_WAIT_FOR_BTN_DOWN;
			sevseg.setChars(allDashesStr);
			button_debounce_time_ms = 0;
			is_new_record = false;
			is_displ_blank = false;
		}
		/* blink display for 5 secs with new record score */
		else
		{
			if(millis() - blink_time_ms >= 250 && is_new_record)
			{
				blink_time_ms = millis();
				if(!is_displ_blank)
				{
					sevseg.blank();
					is_displ_blank = true;
				}
				else
				{
					dispaly_counts(num);
					is_displ_blank = false;
				}
			}
		}
		break;
	}
	}

    sevseg.refreshDisplay(); // Must run repeatedly; don't use blocking code (ex: delay()) in the loop() function or this won't work right
}


void dispaly_counts(uint32_t cnt)
{
	if (cnt <= 9999UL)
	{
		sevseg.setNumber(cnt, 3);
	}
	else if (cnt <= RECORD_MAX)
	{
		sevseg.setNumber(cnt/10, 2);
	}
	else
	{
		/* overflow */
		sevseg.setChars(allDashesStr);
	}
}

/* TODO:
 * - consistent behavior after reset and after each measurement (showing dashes)
 * - record shall be crc-protected
 *  - state machine refactor - loop() is too long
 */
