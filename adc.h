/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

#include <avr/io.h>

#define INT_REF (1<<REFS1)|(1<<REFS0)
#define REF_VOLT 1100 //mV

#define CH0 0x00
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03
#define CH4 0x04
#define CH5 0x05

#define NO_CH 0xFF

void StartAD(uint8_t chan);

#define BAD_CH 0xF000

uint16_t ReadAD(uint8_t chan);


#endif // ADC_H_INCLUDED
