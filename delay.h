/****************************************************************************
*
* Authors:    O??
* Copyright:  ???
*
*
*****************************************************************************/
#ifndef _delay_h_
#define _delay_h_

#include <avr/io.h>

#define fosc1 (F_CPU / 60000)

#if (fosc1 > 255)
#undef  fosc1
#define fosc1 255
#endif

#define fosc2 (F_CPU / 6000000)


void waitms  (unsigned int c);
void waitus  (unsigned int c);
#endif
