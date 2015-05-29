/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/

#include "buttons.h"
#include "delay.h"
#include "global.h"



uint8_t butt_old;



void InitButtons(void){

BUT_DDR &=~(OK|LEFT|RIGHT);
BUT_ESC_DDR &=~(ESC);
BUT_PULL |=(OK|LEFT|RIGHT);
BUT_ESC_PULL |=(ESC);
}


uint8_t TimeState=0;

uint8_t RetButtons(void){

    uint8_t i;
    uint8_t  butt = 0;

    for(i=0;i<10;i++) {
        butt |= ~BUT_PIN&(OK|LEFT|RIGHT);
        butt |= ~BUT_ESC_PIN&(ESC);
        waitms(5);
    }


    if(butt!=0){

        if(CompWdTimer(TimeState,BUT_DELAY)){

            TimeState = RetWdTimer();
            butt_old=butt;


        }else{

            butt &=~butt_old;

        }


    }

    return butt;

}

