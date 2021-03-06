/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
* Upravy
* 2013-08-11 sleep_flag presunuta do car_meas.h
*****************************************************************************/
//#define USART_DEBUG

#include "pwr_down.h"
#include "car_meas.h"

#ifdef USART_DEBUG
#include "uart.h"
#include <stdio.h>
#include <avr/pgmspace.h>
#endif

//uint8_t EEMEM sleep_flag;
volatile uint8_t reboot = 0;


void SetSleep(void){
#ifdef USART_DEBUG
char debug[50];
#endif // USART_DEBUG
    DDRB = 0;
    DDRD = 0;

    PORTB =1;		/* tlacitko OK */
    PORTD = 1 << PD3; /* pullup na OC2B */
    TCCR2B = 0; /* stop timer */
    PORTC=0;/* na portu C jsou tlacitka a adc ty nenulovat */
#ifdef USART_DEBUG
    uart_init(UART_BAUD_SELECT(19200,F_CPU));
    uart_puts_P("DEBUG:SetSleep() provadeni\n");
    sprintf_P(debug,PSTR("MCUSR=%u\nWDTCSR=%u\n"),MCUSR,WDTCSR);
    uart_puts(debug);
    sprintf_P(debug,PSTR("DDRB=%u,PORTB=%u\n"),DDRB,PORTB);
    uart_puts(debug);
    sprintf_P(debug,PSTR("DDRC=%u,PORTC=%u\n"),DDRC,PORTC);
    uart_puts(debug);
    sprintf_P(debug,PSTR("DDRD=%u,PORTD=%u\n"),DDRD,PORTD);
    uart_puts(debug);
#endif // USART_DEBUG

    #ifdef TANK_IMP_CNT
    EIMSK &= ~(1<<INT0);
    #endif
    DisWdTimer();


    StoreCMeas();

    if(bit_is_set(PCIFR,PCIF0)) PCIFR |= 1 << PCIF0;
    if(bit_is_set(PCIFR,PCIF1)) PCIFR |= 1 << PCIF1;
    if(bit_is_set(PCIFR,PCIF2)) PCIFR |= 1 << PCIF2;


    PCICR = 1 << PCIE2 | 1 << PCIE0 ; // PD5 T1 and PC2 PC3
    PCMSK2 = 1 << PCINT21;      // PD5
    PCMSK0 = 1 << PCINT0; // PB0 tlacitko OK

    disComparator();

    ADCSRA=0;
    #ifndef USART_DEBUG
    PRR = 255;
    #else
    PRR=253;

    #endif

//    eeprom_update_byte(&sleep_flag,1);
    setSleepFlag();

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:SetSleep() kompletni \n");
#endif // USART_DEBUG

}

void CheckWakeUp(void){

    uint8_t i=0;
#ifdef USART_DEBUG
    char debug[30];
#endif // USART_DEBUG

    while(bit_is_set(PINB,PB0) /*&& bit_is_set(PINC,PC3) && bit_is_set(PINC,PC4) && bit_is_set(PINC,PC5)*/ && bit_is_set(PIND,PD5) && (i < 200)  ){
        waitus(500);
        i++;
    }

#ifdef USART_DEBUG
    sprintf_P(debug,PSTR("DEBUG:CheckWakeUp i=%u\n"),i);
    uart_puts(debug);
#endif // USART_DEBUG

    if(i<200){

    PCICR = 0;
    PCMSK2 = 0;
    PCMSK0 = 0;



    DisSleep();

    SetReboot();

    }

}

ISR(PCINT0_vect){


    CheckWakeUp();

}

ISR(PCINT2_vect){


    CheckWakeUp();


}

inline void SleepPoll(void){

    do{

   // if(eeprom_read_byte(&sleep_flag)!=1) return;
    if(isSleeFlagEnabled()!=1) return;

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:Sleeping..\n");
#endif // USART_DEBUG
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();

    }while(1);

}

inline void DisSleep(void){

sleep_disable();
//eeprom_update_byte(&sleep_flag,0);
 clearSleepFlag();

}

inline void SetReboot(void){

reboot = 1 ;

}

inline uint8_t CheckReboot(void){

if(reboot){

    reboot = 0;

    return 1;
}

return 0;
}
