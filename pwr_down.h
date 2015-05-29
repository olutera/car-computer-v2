/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef PWR_DOWN_H_INCLUDED
#define PWR_DOWN_H_INCLUDED

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "wdt_timer.h"
#include "car_meas.h"
#include "temp.h"
#include "buttons.h"

#include <stdio.h>
#include "hd44780.h"
#include "hd44780_hw.h"

#include "delay.h"

void SetSleep(void); // nastavi sleep flag

void SleepPoll(void);   // v main testuje zda prejit na sleep

void DisSleep(void);    // zakaze sleep flag

uint8_t CheckReboot(void);
#endif // PWR_DOWN_H_INCLUDED
