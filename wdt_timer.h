/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef WDT_TIMER_H_INCLUDED
#define WDT_TIMER_H_INCLUDED

#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "car_meas.h"

/* cca 120ms ISR wd timer -> max 120*256 ms */

void InitWdTimer(void);
void DisWdTimer(void);
/* return timer cnt*/
uint8_t RetWdTimer(void);

void SetReboot(void);

void EnWdtInit (void);

uint8_t CompWdTimer(uint8_t TimeState,uint8_t delay);

#endif // WDT_TIMER_H_INCLUDED
