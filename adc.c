/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/

#include "adc.h"

uint8_t curr_chnl=NO_CH;

void StartAD(uint8_t chan){

    if(chan == NO_CH) return;

    while(bit_is_set(ADCSRA,ADSC));     /*  pockej pokud bezi prevod */

    ADMUX = INT_REF | chan;

    ADCSRA= 1<< ADEN | 1<<ADSC | 1<< ADPS2  | 1 <<ADPS0;     /*500kHz*/


    curr_chnl = chan;
}

uint16_t ReadAD(uint8_t chan){


    if(curr_chnl == chan){

        while(bit_is_clear(ADCSRA,ADIF));

        curr_chnl = NO_CH;

        return ADC;
    }
    else return BAD_CH;

}
