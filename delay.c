/****************************************************************************
*
* Authors:    ???
* Copyright:  ???
*
*
*****************************************************************************/

#include "delay.h"

void waitms  (unsigned int c)
{
	asm("push	r20");
	asm("push	r21");
asm("_Wms0:");
	asm("ldi	r20,0x14");
asm("_Wms1:");
	asm volatile("ldi r21,%0" : : "M" (fosc1));
asm("_Wms2:");
	asm("dec	r21");
	asm("brne	_Wms2");
	asm("dec		r20");
	asm("brne	_Wms1");
	asm("dec	r24");
	asm("brne	_Wms0");
	asm("dec	r25");
	asm("brpl	_Wms0");
	asm("pop   	r21");
	asm("pop   	r20");
}


void waitus(unsigned int c)
{
	asm("push	r20");
asm("_wus0:");
	asm volatile("ldi r20,%0" : : "M" (fosc2));
asm("_wus1:");
	asm("dec	r20");
	asm("nop");
	asm("nop");
	asm("brne	_wus1");
	asm("dec	r24");
	asm("nop");
	asm("brne	_wus0");
	asm("dec	r25");
	asm("nop");

	asm("brpl	_wus0");
	asm("pop	r20");
}
