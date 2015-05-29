/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#include "wdt_timer.h"

/*  250 ms ISR */
volatile uint8_t wdt_timer=0;
uint8_t wdt_tim = 0;
volatile uint8_t wdt_init=0;


 ISR(WDT_vect){

     WDTCSR |= 1<<WDCE | 1 <<WDIE;

    wdt_tim++;

    if(wdt_tim==4/*CompWdTimer(wdt_tim_old,4)*/) { // kazdou 1s odber dat

        CMeasPoll();

       wdt_tim=0;
    }

    wdt_timer++;


 }

void InitWdTimer(void){
cli();
 if(wdt_init != 1)   {



 wdt_enable(WDTO_250MS);

 WDTCSR |= 1<<WDCE | 1 <<WDIE;



 wdt_init = 1;

 }

sei();
}

void DisWdTimer(void){
cli();

wdt_disable();
MCUSR &= ~(1<<WDRF);
WDTCSR |= (1<<WDCE) | (1<<WDE);
WDTCSR = 0x00;

wdt_init = 0;

sei();
}

uint8_t RetWdTimer(void){

return wdt_timer;

}


uint8_t CompWdTimer(uint8_t TimState,uint8_t delay){

    uint8_t ActTimer = RetWdTimer();

    if(TimState > ActTimer && (255 - TimState + ActTimer) > delay) return 1;

    if(TimState < ActTimer && (ActTimer - TimState) > delay) return 1;


    return 0;
}


