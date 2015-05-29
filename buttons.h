/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef BUTTONS_H_INCLUDED
#define BUTTONS_H_INCLUDED

#include <avr/io.h>
#include "wdt_timer.h"

#define BUT_DELAY 1

#define BUT_DDR     DDRB
#define BUT_PULL    PORTB
#define BUT_PIN     PINB

#define BUT_ESC_DDR DDRD
#define BUT_ESC_PULL PORTD
#define BUT_ESC_PIN	PIND

#define OK      (1<<PB0)
#define ESC     (1<<PD7)
#define RIGHT   (1<<PB2)
#define LEFT    (1<<PB1)

void InitButtons(void);

uint8_t RetButtons(void);
#endif // BUTTONS_H_INCLUDED
