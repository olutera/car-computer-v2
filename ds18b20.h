/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef DS18B20_H_INCLUDED
#define DS18B20_H_INCLUDED

#include <avr/io.h>
#include "delay.h"

#define DPIN PIND		// def vstupniho reg /PORT
#define DPORT PORTD		// vystupni reg		/PORT
#define DDDR DDRD		// prepinani vstup/vystup  /PORT
#define DAT1 0			// pin daneho portu
#define DAT2 1			// druhe cidlo pin dalsiho portu


void init_dallas(char pin);
void send(uint8_t data,char pin);
void read(int16_t * temp,char pin);
void strong_pull(char state,char pin);


#define startT(temp_pin) init_dallas(temp_pin);send(0xCC,temp_pin);send(0x44,temp_pin);strong_pull(1,temp_pin);		// spust mereni (nutno cekat cca 500ms)
#define readT(A,temp_pin) strong_pull(0,temp_pin);init_dallas(temp_pin);send(0xCC,temp_pin);send(0xBE,temp_pin);read(A,temp_pin); //vrat namer. teplotu

#define startT_out startT(DAT1)
#define startT_in startT(DAT2)

#define readT_out(A) readT(A,DAT1)
#define readT_in(A) readT(A,DAT2)

#endif // DS18B20_H_INCLUDED
