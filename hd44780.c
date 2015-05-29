/*
 * Title    :   HD44780U LCD library
 * Authors:
 * Based on Volker Oth's lcd library (http://members.xoom.com/volkeroth)
 * modified by Peter Fleury's (http://jump.to/fleury). Flexible pin
 * configuration by Markus Ermert. Adapted for the linuxfocus LCD display
 * by Guido Socher.Binded to be used with stdio.h by Fandi Gunawan
 * using lcd_putc_stream
 *
 * Software:  AVR-GCC with AVR-AS (old and new is OK since there is avr_compat.h)
 * Target:    any AVR device
 * Copyright: GPL
 */

/*
** C standard library
*/
//#include <stdbool.h>      // edit Luta
//#include <stdint.h>
#include <stdio.h>

/*
** AVR library
*/
#include <avr/io.h>
#include <avr/pgmspace.h>


#include "delay.h"      // Edit Luta, _delay_us -> waitus

#include "hd44780_hw.h"
#include "hd44780.h"
#include "avr_compat.h"

#ifdef USART_DEBUG
#include "uart.h"
char debug[200];
#endif // USART_DEBUG

/*
** constants/macros
*/

#define lcd_e_high()    sbi(LCD_E_PORT, LCD_E_PIN)
#define lcd_e_low()     cbi(LCD_E_PORT, LCD_E_PIN)
#define lcd_e_toggle()  toggle_e()

#define lcd_cmd_mode()  cbi(LCD_RS_PORT, LCD_RS_PIN)    /* RS=0  command mode   */
#define lcd_data_mode() sbi(LCD_RS_PORT, LCD_RS_PIN)    /* RS=1  data mode      */
#define lcd_wr_mode()   cbi(LCD_RW_PORT, LCD_RW_PIN)    /* RW=0  write mode     */
#define lcd_rd_mode()   sbi(LCD_RW_PORT, LCD_RW_PIN)    /* RW=1  read mode      */
#define lcd_data_port_in()  \
{   /* defines all data pins as input */ \
    cbi(LCD_DATA_DDR_D7,LCD_DATA_PIN_D7);\
    cbi(LCD_DATA_DDR_D6,LCD_DATA_PIN_D6);\
    cbi(LCD_DATA_DDR_D5,LCD_DATA_PIN_D5);\
    cbi(LCD_DATA_DDR_D4,LCD_DATA_PIN_D4);\
}
#define lcd_data_port_out() \
{   /* defines all data pins as output */ \
    sbi(LCD_DATA_DDR_D7,LCD_DATA_PIN_D7);\
    sbi(LCD_DATA_DDR_D6,LCD_DATA_PIN_D6);\
    sbi(LCD_DATA_DDR_D5,LCD_DATA_PIN_D5);\
    sbi(LCD_DATA_DDR_D4,LCD_DATA_PIN_D4);\
}

#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES
#endif

/*
** function prototypes
*/
 void toggle_e(void);
 void lcd_out_high(u08 d);
 void lcd_out_low(u08 d);

/*
** local functions
*/

/*
** delay for a minimum of <ms>
*/
inline void lcdwaitms(unsigned int ms)
{
        /* we use a calibrated macro. This is more
        ** accurate and not so much compiler dependent
        ** as self made code
        */

        /* Edit Luta */
        waitms(ms+1);
/*
        while(ms){
                //_delay_loop_2(240.0);
                waitus(1000);
                ms--;
        }
*/		
        //waitms(ms);
}

inline void lcdwaitus(unsigned int us)
{
        waitus(us+1);
}

/*
** output low nibble
*/
 void lcd_out_low(u08 d)
{
    if (d&0x08)  sbi(LCD_DATA_PORT_D7,LCD_DATA_PIN_D7);
        else cbi(LCD_DATA_PORT_D7,LCD_DATA_PIN_D7);
    if (d&0x04)  sbi(LCD_DATA_PORT_D6,LCD_DATA_PIN_D6);
        else cbi(LCD_DATA_PORT_D6,LCD_DATA_PIN_D6);
    if (d&0x02)  sbi(LCD_DATA_PORT_D5,LCD_DATA_PIN_D5);
        else cbi(LCD_DATA_PORT_D5,LCD_DATA_PIN_D5);
    if (d&0x01)  sbi(LCD_DATA_PORT_D4,LCD_DATA_PIN_D4);
        else cbi(LCD_DATA_PORT_D4,LCD_DATA_PIN_D4);
}

/*
** output high nibble
*/
 void lcd_out_high(u08 d)
{


    if (d&0x80)  sbi(LCD_DATA_PORT_D7,LCD_DATA_PIN_D7);
        else cbi(LCD_DATA_PORT_D7,LCD_DATA_PIN_D7);
    if (d&0x40)  sbi(LCD_DATA_PORT_D6,LCD_DATA_PIN_D6);
        else cbi(LCD_DATA_PORT_D6,LCD_DATA_PIN_D6);
    if (d&0x20)  sbi(LCD_DATA_PORT_D5,LCD_DATA_PIN_D5);
        else cbi(LCD_DATA_PORT_D5,LCD_DATA_PIN_D5);
    if (d&0x10)  sbi(LCD_DATA_PORT_D4,LCD_DATA_PIN_D4);
        else cbi(LCD_DATA_PORT_D4,LCD_DATA_PIN_D4);


}

/*
** toggle Enable Pin
*/
 void toggle_e(void)
{
    waitus(5); /* add LS*/
    lcd_e_high();
    waitus(5); /* edit luta - default 2*/
    lcd_e_low();
    waitus(5); /* edit luta - default 2*/
}

/*
** write data to lcd
*/
 void lcd_write(u08 data, u08 rs)
{
    /* configure data pins as output */
    lcd_data_port_out();

    /* output high nibble first */

    lcd_out_high(data);

    if (rs)
        lcd_data_mode();    /* RS=1: write data        */
    else
        lcd_cmd_mode();     /* RS=0: write instruction */
    lcd_wr_mode();          /* RW=0  write mode        */
    lcd_e_toggle();

    /* output low nibble */
    lcd_out_low(data);

    if (rs)
        lcd_data_mode();    /* RS=1: write data       */
    else
        lcd_cmd_mode();     /* RS=0: write instruction*/
    lcd_wr_mode();          /* RW=0  write mode       */

    lcd_e_toggle();

    /* all data pins high (inactive) */
    lcd_data_port_in();
}

/*
** read from LCD
*/
 u08 lcd_read(u08 rs)
{
    register u08 data;

    /* configure data pins as input */
    lcd_data_port_in();

    if (rs)
        lcd_data_mode();    /* RS=1: read data      */
    else
        lcd_cmd_mode();     /* RS=0: read busy flag */
    lcd_rd_mode();          /* RW=1  read mode      */


    lcd_e_high();
    waitus(10); /* luta 2*/

    /* read high nibble first */

    data  = bit_is_set(LCD_DATA_PINR_D7, LCD_DATA_PIN_D7) ? 0x80 : 0;
    data |= bit_is_set(LCD_DATA_PINR_D6, LCD_DATA_PIN_D6) ? 0x40 : 0;
    data |= bit_is_set(LCD_DATA_PINR_D5, LCD_DATA_PIN_D5) ? 0x20 : 0;
    data |= bit_is_set(LCD_DATA_PINR_D4, LCD_DATA_PIN_D4) ? 0x10 : 0;



    /* Enable low*/
    lcd_e_low();
    waitus(10);/* luta 2*/

    lcd_e_high();
    waitus(10);/* luta orig 2*/

    /* read low nibble */

    data |=bit_is_set(LCD_DATA_PINR_D7, LCD_DATA_PIN_D7) ? 0x08 : 0;
    data |=bit_is_set(LCD_DATA_PINR_D6, LCD_DATA_PIN_D6) ? 0x04 : 0;
    data |=bit_is_set(LCD_DATA_PINR_D5, LCD_DATA_PIN_D5) ? 0x02 : 0;
    data |=bit_is_set(LCD_DATA_PINR_D4, LCD_DATA_PIN_D4) ? 0x01 : 0;


    lcd_e_low();

    return (data);
}

/*
** loops while lcd is busy, reads address counter
*/
unsigned char lcd_waitbusy(void)
{
    register unsigned char c;

    while ((c = lcd_read(0)) & (1 << LCD_BUSY)) ; /* polling */

    return (c); /* return address counter=position */
}


/*
** PUBLIC FUNCTIONS
*/


/*
** send command <cmd> to LCD
*/
void lcd_command(u08 cmd)

{
    lcd_waitbusy();
    lcd_write(cmd, 0);
}

/*
** goto position (x,y)
*/
void lcd_gotoxy(u08 x, u08 y)
{

#if LCD_LINES==1
    lcd_command((1 << LCD_DDRAM) + LCD_START_LINE1 + x);
#endif

#if LCD_LINES==2
    if (y == 0)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE1 + x);
    else
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE2 + x);
#endif

#if LCD_LINES==3
    if (y == 0)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE1 + x);
    else if (y == 1)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE2 + x);
    else if (y == 2)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE3 + x);
#endif

#if LCD_LINES==4
    if (y == 0)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE1 + x);
    else if (y == 1)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE2 + x);
    else if (y == 2)
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE3 + x);
    else            /* y==3 */
        lcd_command((1 << LCD_DDRAM) + LCD_START_LINE4 + x);
#endif
}

/*
** clear lcd and set cursor to home position
*/
void lcd_clrscr(void)
{
    lcd_command(1 << LCD_CLR);
}

/*
** set cursor to home position
*/
void lcd_home(void)
{
    lcd_command(1 << LCD_HOME);
}

/*
** print character at current cursor position
*/
unsigned char lcd_putc(char c)
{
    static unsigned char ret_val;
    ret_val=lcd_waitbusy(); /* read busy-flag and address counter */
    lcd_write((unsigned char)c, 1);
    return ret_val;
}

/*
** binding function of lcd_put_c to be used with stdio.h functions
** such as fprintf or printf
*/
int lcd_putc_stream(char c, FILE *stream)
{
    static unsigned char address_counter;
    static uint8_t next_line;

#if LCD_LINES == 1
    if( next_line && c!='\n')
    {
        next_line = 0;
        lcd_clrscr(); /* clear lcd and goto home */
    }
    if( address_counter == (LCD_START_LINE1+LCD_CHARS))
    {
        lcd_clrscr();
    }
#endif

#if LCD_LINES == 2
    if((address_counter < (LCD_START_LINE1+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,1); /*goto next line */
    }
    if((address_counter > (LCD_START_LINE1+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_clrscr(); /* clear lcd and goto home */
    }
    if(address_counter == (LCD_START_LINE1+LCD_CHARS))
    {
        lcd_gotoxy(0,1); /*goto next line */
    }
    if(address_counter == (LCD_START_LINE2+LCD_CHARS))
    {
        lcd_clrscr(); /* clear lcd and goto home */
    }
#endif

#if LCD_LINES == 3
    if((address_counter < (LCD_START_LINE1+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,1);
    }
    if(address_counter == (LCD_START_LINE1+LCD_CHARS))
    {
        lcd_gotoxy(0,1);
    }
    if((address_counter > (LCD_START_LINE1+LCD_CHARS)) && \
        (address_counter < (LCD_START_LINE2+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,2);
    }
    if(address_counter == (LCD_START_LINE2+LCD_CHARS))
    {
        lcd_gotoxy(0,2);
    }
    if((address_counter > (LCD_START_LINE2+LCD_CHARS)) && \
        (address_counter < (LCD_START_LINE3+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_clrscr();
    }
    if(address_counter == (LCD_START_LINE3+LCD_CHARS))
    {
        lcd_clrscr();
    }
#endif

#if LCD_LINES == 4
    if((address_counter < (LCD_START_LINE1+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,1);
    }
    if(address_counter == (LCD_START_LINE1+LCD_CHARS))
    {
        lcd_gotoxy(0,1);
    }
    if((address_counter > (LCD_START_LINE1+LCD_CHARS)) && \
        (address_counter < (LCD_START_LINE2+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,2);
    }
    if(address_counter == (LCD_START_LINE2+LCD_CHARS))
    {
        lcd_gotoxy(0,2);
    }
    if((address_counter > (LCD_START_LINE2+LCD_CHARS)) && \
        (address_counter < (LCD_START_LINE3+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_gotoxy(0,3);
    }
    if(address_counter == (LCD_START_LINE3+LCD_CHARS))
    {
        lcd_gotoxy(0,3);
    }
    if((address_counter > (LCD_START_LINE3+LCD_CHARS)) && \
        (address_counter < (LCD_START_LINE4+LCD_CHARS)) && \
        next_line && c!='\n')
    {
        next_line = 0;
        lcd_clrscr();
    }
    if(address_counter == (LCD_START_LINE4+LCD_CHARS))
    {
        lcd_clrscr();
    }
#endif
    if(c=='\n')
    {
        next_line=1;
    }
    else
    {
        if(c=='\0')
            return 0;
        else
            address_counter=lcd_putc(c);
    }

    return 0;
}

/* edit Luta */
/*
void rolltext_lcd_p(const char *text, int shiftlen){
        unsigned char i;
        unsigned char j;
        unsigned char textlen=0;
        while (pgm_read_byte(text+textlen)){
                textlen++;
        }
        if (shiftlen <= ROLLLEN){
        i=0;
                j=ROLLLEN-shiftlen + 1;
                while(i < j){
                        lcd_putc(' ');
                        i++;
                }
        j=0;
        }else{
                j=shiftlen-ROLLLEN;
        i=0;
        }
        if (j <= textlen) {
                while(i < ROLLLEN){
                        if (pgm_read_byte(text+j)=='\0') break;
                        lcd_putc(pgm_read_byte(text+j));
                        j++;
                        i++;
                }
        }
}
*/
/*
** print string on lcd  with no auto line feed
*/
void lcd_puts(const char *s)
{
    while (*s) {
        lcd_putc(*s);
        s++;
    }
}

/*
** print string from program memory on lcd
*/
void lcd_puts_p(const char *progmem_s)
{
    char c;

    while ((c = pgm_read_byte(progmem_s++))) {
        lcd_putc(c);
    }

}

/*
** initialize display and select type of cursor
** dispAttr: LCD_DISP_OFF, LCD_DISP_ON, LCD_DISP_ON_CURSOR, LCD_DISP_CURSOR_BLINK
*/
void lcd_init(u08 dispAttr)
{
    /*------ Initialize lcd to 4 bit i/o mode -------*/

#ifdef USART_DEBUG
uart_puts_P("DEBUG:lcd_init()\n");
#endif // USART_DEBUG
    lcd_data_port_out();    /* all data port bits as output */
    sbi(LCD_RS_DDR, LCD_RS_PIN);    /* RS pin as output */
    sbi(LCD_RW_DDR, LCD_RW_PIN);    /* RW pin as output */
    sbi(LCD_E_DDR, LCD_E_PIN);  /* E  pin as output */
	// LS 2014-11-16 - set all data levels to low
	lcd_out_low(0x00);
	lcd_out_high(0x00);

    /* lcd power on */
    DDRD |= 1 << PD2;
    PORTD &= ~(1 << PD2);
    lcdwaitms(50);   /* wait 16ms or more after power-on       */ // > 100ms - pockat, az kompletne probehne inicializace po zapnutí
	                 //  15ms after VDD > 4,5V, 40ms after VDD > 2,7V

#ifdef USART_DEBUG
uart_puts_P("DEBUG:lcd_init() - pin setup complete\n");
#endif // USART_DEBUG
    /* initial write to lcd is 8bit */
    lcd_out_high(0x30 /*LCD_FUNCTION_8BIT_2LINES*/);
    lcd_e_toggle();
    lcdwaitms(5);    /* delay, busy flag can't be checked here */

    lcd_out_high(0x30 /*LCD_FUNCTION_8BIT_2LINES*/);
    lcd_e_toggle();
    lcdwaitus(200); //lcdwaitms(5);    /* delay, busy flag can't be checked here */

    lcd_out_high(0x30 /*LCD_FUNCTION_8BIT_2LINES*/);
    lcd_e_toggle();
    lcdwaitus(200); //lcdwaitms(5);    /* delay, busy flag can't be checked here */

#ifdef USART_DEBUG
uart_puts_P("DEBUG:lcd_init() - initialized 8bit\n");
#endif // USART_DEBUG
    lcd_out_high(0x20 /*LCD_FUNCTION_4BIT_2LINES*/);  /* set IO mode to 4bit */
    lcd_e_toggle();

#ifdef USART_DEBUG
uart_puts_P("DEBUG:lcd_init() - initialized 4bit\n");
#endif // USART_DEBUG
    /* from now the lcd only accepts 4 bit I/O, we can use lcd_command() */
    lcd_command(0x20 /*LCD_FUNCTION_DEFAULT*/);  /* function set: display lines  */
    lcd_command(LCD_DISP_OFF);  /* display off                  */
    lcd_clrscr();       /* display clear                */
    lcd_command(LCD_MODE_DEFAULT);  /* set entry mode               */
    lcd_command(dispAttr);  /* display/cursor control       */
}
/* add by Luta */
void lcd_cgram_write(const char * array8,uint8_t addr){
uint8_t tmp,i;
	addr*=8;

	addr &= ~_BV(7);

	addr |= _BV(6);

	lcd_command(addr);

	for(i=0;i<8;i++){
	tmp = pgm_read_byte(array8+i);
	lcd_putc(tmp);
	}

}
