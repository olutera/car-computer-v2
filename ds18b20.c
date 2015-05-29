/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/


#include "ds18b20.h"

#define setb(reg,bit) reg |= _BV(bit)
#define clrb(reg,bit) reg &= ~ _BV(bit)

void init_dallas(char pin){

clrb(DPORT,pin);

setb(DDDR,pin);
waitus(500);

clrb(DDDR,pin);
waitus(70);
if(bit_is_clear(DPIN,pin))waitus(410);

}

void strong_pull(char state,char pin){

    if(state){
        setb(DPORT,pin);
        setb(DDDR,pin);
    }else{
        clrb(DDDR,pin);
        clrb(DPORT,pin);

    }

}

void send(uint8_t data,char pin){
uint8_t i;


clrb(DPORT,pin);
for(i=0;i<8;i++){
	if(bit_is_set(data,i)){

			setb(DDDR,pin);
			waitus(6);

			clrb(DDDR,pin);
			waitus(64);
	}
	else{

			setb(DDDR,pin);
			waitus(60);

			clrb(DDDR,pin);
			waitus(10);
		}
}


}
/* vraci teplotu cidla na pinu jako 255 = 25.5 °C */
void read(int16_t * p,char pin){
uint8_t i;
*p=0;

clrb(DPORT,pin);
for(i=0;i<16;i++){

	setb(DDDR,pin);
	waitus(6);

	clrb(DDDR,pin);
	waitus(9);
	if(bit_is_set(DPIN,pin)) *p|=(1<<i) ;


	waitus(55);
}

*p = *p*5/8;

}



