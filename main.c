/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>

//#define USART_DEBUG


#include "hd44780.h"
#include "hd44780_hw.h"
#include "temp.h"
#include "wdt_timer.h"
#include "buttons.h"
#include "adc.h"
#include "car_meas.h"
#include "pwr_down.h"
#include "delay.h"
#include "string.h"
#include "version.h"

#ifdef USART_DEBUG
#include "uart.h"
#endif // USART_DEBUG

/* definice MAX a MIN pro SETUP MENU a jednotlive poradi a cislovani MENU - je pouzit spolecna promenna proto je setup menu s offsetem +100 */

#define MINSMENU 100
#define MAXSMENU 115

#define EDIT_INJ  100       // editace konstanty vstrikovace v ccm/min
#define EDIT_VALVE 101      // editace poctu valcu motoru
#define EDIT_IMP  102       // editace poctu impulzu na 100m
#define EDIT_TANK_L 103     // editace objemu nadrze
#define EDIT_TANK_MODE 104
#define EDIT_TANK1 105      // prvni kalib. konstanty napeti v mV a litry nadrze   -- nadrz pocita pres linearni fci -- sestupna ci rostouci dle konstant
#define EDIT_TANK2 106      // druha skupina kalib. konstanty napeti v mV a litry nadrze
#define EDIT_TANK3 107
#define EDIT_TANK4 108
#define EDIT_TANK5 109
#define EDIT_SENSORS 110
#define EDIT_SLEEP_TIME 111
#define EDIT_CALIB_VOLT 112
#define EDIT_MENU1  113
#define EDIT_RESET 114     // kompletni reset vymazani
#define EDIT_SHOW  115

/* Definice MENU pro zobrazeni provoznich hodnot. Zde menit poradi editaci cisel ! */

#define MAXMENU 6
#define MINMENU 0
/*
#define AKTUSP      0           // aktualni spotreba        xx,x l/100km ci xx,x l/h pokud vuz stoji
#define PRUMSP      1           // prumerna spotreba na danou aktualni jizdu    xx,x l/100km

#define TRASSP      2           // trasa spotreba - mazatelna az uzivatelem     xx,x l/100km
#define DOJEZD      3           // dojezd pocitany z prumerne spotreby a stavu nadrze   xxx km

#define TRAVYJ      4           // trasa vyjete litry benzinu       xx L
#define AKTVYJ      5           // aktualne vyjete litry benzinu po aktualni jizdu xx L

#define AKTUUJ      6           // aktualne ujeta vzdalenost ve formatu xxxx,x km
#define TRASUJ      7          // trasa ujeta vzdalenost format xxxx,x km

#define TEPVEN      8           // teplota venkovni cidlo format xx,x °C
#define TEPVNI      9              // teplota vnitrni cidlo format xx,x °C

#define NAPETI      10          // napeti baterie - format xx,x V
#define NADRZ       11          // stav nadrze format xx L

#define CELKUJ      12          // celkove ujete km ve formatu xxxxxxxxx km - mazatelne pouze kompletnim resetem
#define CELKSP      13          // celkova spotreba ve fornatu xx,x l/100km
*/
/* casovani prechodu do power down + doba drzeni OK a ESC pro vymazani Trip dat */

#define SLEEP_TIME 4*60     /* 30s */


#define CLEAR_TRIP_TIME 10

const char PROGMEM LcdStart1[]=    "Palubni pocitac ";
const char PROGMEM LcdStartVer[] = "v.";

/* text popisky ulozene ve flash */


/*  const char PROGMEM CelkUj[] = "C.uj";
  const char PROGMEM CelkSp[] = "C.sp";
  //PGM_P   CelkSp = CelkUj;

  const char PROGMEM TrasUj[] = "T.uj";
  const char PROGMEM TrasSp[] = "T.sp";
  //PGM_P TrasSp = TrasUj;

  const char PROGMEM AktuSp[] = "A.sp";

  const char PROGMEM PrumSp[] = "P.sp";


  const char PROGMEM AktuUj[] = "A.uj";
//  PGM_P AktuUj = AktuSp;

  const char PROGMEM AktuVyj[] ="A.vy";
  //PGM_P AktuVyj = AktuSp;

  const char PROGMEM TepVen[] = "Venk";
  const char PROGMEM TepVni[] = "Vnit";

  const char PROGMEM Napeti[] = "Nap ";
  const char PROGMEM Dojezd[] = "Doj ";


  const char PROGMEM TraVyj[] = "T.vy";
  //PGM_P TraVyj = TrasUj;

  const char PROGMEM Nadrz[]  = "Nadr"; */

const char PROGMEM Nastaveni[] ="    Nastaveni   ";
const char PROGMEM EmptyLine[] ="                ";

const char PROGMEM KonstInj[] = "Vstrik.konstanta";
const char PROGMEM KonstImp[] = "Metry  konstanta";
//const char PROGMEM Tank1Kalib[] ="Kalib. nadrz L1 ";
//const char PROGMEM Tank2Kalib[] ="Kalib. nadrz L2 ";
const char PROGMEM TankKalib[] ="Kalib. nadrz L%u";

const char PROGMEM TankObj[] =  "Objem pal. nadrz";
const char PROGMEM CelkReset[] ="Celkovy reset PP";
const char PROGMEM Editace[]   ="Editace";
const char PROGMEM KonstValve[] = "  Vstrikovacu:";

const char PROGMEM ZmeritNapeti[] = "Zmerit napeti?\nESC=NE    OK=ANO";
const char PROGMEM KomplReset[] = "Kompletni reset?\nESC=NE    OK=ANO";
const char PROGMEM Vymazano[] = "Vymazano!";
const char PROGMEM Merim[] = "Merim";
const char PROGMEM EditSensors[] = "Teplotni cidla: ";
const char PROGMEM text_no_sensor[] = "Zadne cidlo     ";
const char PROGMEM text_out_sensor[] = "Cidlo c.1       ";
const char PROGMEM text_in_sensor[] =  "Cidlo c.2       ";
const char PROGMEM text_out_in_sensor[] = "Obe cidla       ";
const char PROGMEM TankMode[] = "Rezim nadrze:   ";
const char PROGMEM text_tank_mode_standard[] = "  standardni    ";
const char PROGMEM text_tank_mode_int[] = "  detekce imp.  ";
const char PROGMEM SleepTimeText[] = "Prodleva vypnuti";
const char PROGMEM CalibVoltText[] = "Kalib. nap. site";
const char PROGMEM Menu1Text[]=      "Volba zobrazeni ";
const char PROGMEM text_menu1_0[]=   "     Dojezd     ";
const char PROGMEM text_menu1_1[]=   "   Obe teploty  ";
const char PROGMEM text_menu1_2[]=   "    Teplota     ";
const char PROGMEM text_menu1_3[]=   "     Napeti     ";
const char PROGMEM text_menu1_4[]=   "  Vyjete litry  ";
const char PROGMEM text_menu1_5[]=   "  Automaticky   ";

/* uzivatelske znaky v CGRAM lcd */

#define CHAR_STUPEN 0
#define CHAR_100KM_1 1
#define CHAR_100KM_2 2
#define CHAR_KM 3
#define CHAR_Z 4
#define CHAR_A  5
#define CHAR_IO1 6
#define CHAR_IO2 7

const char PROGMEM char_stupen[8]=
{
    0b00011000,
    0b00011000,
    0b00000110,
    0b00001001,
    0b00001000,
    0b00001001,
    0b00000110,
    0b00000000
};
const char PROGMEM char_100km_1[8]=
{
    0b00010111,
    0b00010101,
    0b00010111,
    0b00000000,
    0b00000101,
    0b00000110,
    0b00000101,
    0b00000000
};
const char PROGMEM char_100km_2[8]=
{
    0b00011100,
    0b00010100,
    0b00011100,
    0b00000000,
    0b00011110,
    0b00010101,
    0b00010101,
    0b00000000
};

const char PROGMEM char_km[8]=
{
    0b00001010,
    0b00001100,
    0b00001010,
    0b00000000,
    0b00011010,
    0b00010101,
    0b00010101,
    0b00000000
};

const char PROGMEM char_z[8]=
{
    0b00001010,
    0b00000100,
    0b00011111,
    0b00000010,
    0b00000100,
    0b00001000,
    0b00011111,
    0b00000000
};

const char PROGMEM char_a[8]=
{
    0b00000010,
    0b00000100,
    0b00001110,
    0b00000001,
    0b00001111,
    0b00010001,
    0b00001111,
    0b00000000
};

const char PROGMEM char_io1[8]=
{
    0b00010110,
    0b00010101,
    0b00010101,
    0b00000000,
    0b00000001,
    0b00000010,
    0b00000100,
    0b00001000
};


const char PROGMEM char_io2[8]=
{
    0b00000010,
    0b00000100,
    0b00001000,
    0b00010000,
    0b00000110,
    0b00001001,
    0b00001001,
    0b00000110
};

uint8_t CurrMenu=MINMENU;       // promenna MENU
uint8_t NoSleep=0;              // blokovani prechodu do power down pokud je zachytnuto tlacitko
uint8_t SleepTimeout = 0;       // pomocna promenna pro sleep


/*headers ----------------------------------------------------------*/

void PrintVal(uint8_t index);
void LcdMenu(void);
void IncMenu(uint8_t min,uint8_t max);
void DecMenu(uint8_t min,uint8_t max);
void print_temp(int16_t temp);
void print_km100(m_t m);
void print_fuel(fuel_t f);
void print_fuelcons(fuel_t f);
void print_V(mv_t mv);
uint16_t EditCalibVal(uint16_t val,uint8_t cnt,uint8_t prec,uint16_t max,char *text);
void LcdBlinkView(char c);
mv_t LcdMeasmV(mv_t mv);
/* functions -------------------------------------------------------*/

/* stara se o pohyb mezi menu hodnotami, pokud je pozadovano prechazeni z nejvyssiho menu do nejnizsiho tak prave zde zmenit */
void DecMenu(uint8_t min,uint8_t max)
{

    if(CurrMenu>min)CurrMenu--;
    else CurrMenu=max;

}

void IncMenu(uint8_t min,uint8_t max)
{

    if(CurrMenu<max)CurrMenu++;
    else CurrMenu=min;

}
/* funkce pro tisk provoznich hodnot a jejich formatovani */
/*
void print_temp(int16_t temp){
int8_t prec = (temp%10);
if (prec<0) prec*=-1;

printf("%7i,%i ",temp/10,prec);
lcd_putc(CHAR_STUPEN);
lcd_putc('C');
}

void print_km100(m_t m){

m/=100;

printf("%7lu,%lu km",m/10,m%10);

}

void print_fuel(fuel_t f){

printf("%5u,%u L",f/10,f%10);

}

void print_fuelL(fuel_t f){

printf("%7u,%u  L",f/10,f%10);

}

void print_fuelcons(fuel_t f){

print_fuel(f);
//lcd_putc('/');
lcd_putc(CHAR_100KM_1);
lcd_putc(CHAR_100KM_2);

}

void print_V(mv_t mv){

printf("%7u,%u  V",mv/1000,(mv%1000)/100);

}
*/
/* tisk provoznich hodnot - pro jine serazeni staci predefinovac cisla u #define maker */
/* TODO - udelat to pres tabulky menitelne jako v prototypu - zatim neni misto ve flash u atmega168*/
/*
void PrintVal(uint8_t index){



    switch(index){

    case AKTUSP:
                    printf_P(AktuSp);

                    if(GetActMeters()>4) {

                     print_fuelcons(GetCurrFuelCons());

                    }else {

                     print_fuel(GetCurrFuelConsL());
                     printf("/h ");

                    }
                    break;
    case PRUMSP:
                    printf_P(PrumSp);

                    print_fuelcons(GetAvgFuelCons());

                    break;

    case TRASSP:
                    printf_P(TrasSp);

                    print_fuelcons(GetTripFuelCons());
                    break;

    case DOJEZD:
                    printf_P(Dojezd);

                    printf("%9u km",GetMaxDis());

                    break;

    case TRAVYJ:    printf_P(TraVyj);

                    print_fuelL(GetTripFuel());


                    break;

    case AKTVYJ:    printf_P(AktuVyj);

                    print_fuelL(GetCurrFuel());


                    break;

    case AKTUUJ:


                    printf_P(AktuUj);

                    print_km100(GetCurrMeters());
                    break;

    case TRASUJ:

                    printf_P(TrasUj);

                    print_km100(GetTripMeters());
                    break;

    case TEPVEN:

                    printf_P(TepVen);

                    print_temp(RetSensorOut());


                    break;

    case TEPVNI:

                    printf_P(TepVni);

                    print_temp(RetSensorIn());

                    break;

    case NAPETI:
                    printf_P(Napeti);

                    print_V(GetV12mV());

                    break;

    case NADRZ :
                    printf_P(Nadrz);

                    printf("%9u  L",GetTankL());

                    break;

    case CELKUJ:
                    printf_P(CelkUj);

                    printf("%9lu km",GetTotMeters()/1000);

                    break;

    case CELKSP:
                    printf_P(CelkSp);

                    print_fuelcons(GetTotFuelCons());
                    break;

    case MAXMENU*2:

                    printf_P(Nastaveni);

                    break;
    case MAXMENU*2+1:

                    printf_P(EmptyLine);
                    break;

    default: break;

    }


}
*/
/* zobrazi na LCD editovaci nabidku hodnoty val o max poctu cislic cnt , desetinych mist prec , editovatelne do maximalni hodnoty max a za hodnotou zobrazi text */

uint16_t EditCalibVal(uint16_t val,uint8_t cnt,uint8_t prec,uint16_t max,char * text)
{

    uint8_t but = RetButtons();
    uint16_t edit = val;
    uint8_t pos=1;
    uint8_t size=1;
    uint16_t dat;
    uint16_t i;

    while(1)
    {
        lcd_command(LCD_DISP_ON);
        if(but&ESC) return val;
        if(but&OK)
        {

            if(pos==cnt+prec)  return edit;
            else pos++;

        }

        dat=1;
        for(i=1; i<pos; i++) dat*=10; // rady cisla

        if(but&LEFT && (edit>=dat)) edit-=dat;      // edit cisla


        if(but&RIGHT && (edit<=(max-dat))) edit+=dat;

        size=0;
        if(edit>(0+10*prec))          // detekce formatovani cisla na LCD
        {

            i=edit;
            while(i>0)
            {
                size++;
                i/=10;

            }

        }
        else size=1+prec;

        lcd_clrscr();
        lcd_gotoxy(0,0);
        printf_P(Editace);

        lcd_gotoxy(7-size-prec,1);
        if(prec!=0)printf("%u,%u %s",edit/(10*prec),edit%(10*prec),text);
        else printf("%u %s",edit,text);

        if(pos>prec && prec!=0)lcd_gotoxy(7-prec-pos,1);
        else lcd_gotoxy(7-pos,1);
        lcd_command(LCD_DISP_ON_CURSOR);
        but = RetButtons();
        waitms(100);

    }


}


void LcdBlinkView(char c)
{

    lcd_gotoxy(4,0);
    lcd_putc(c);
    lcd_gotoxy(4,1);
    lcd_putc(c);


}
/* meri v kalibraci napeti nadrze */
mv_t LcdMeasmV(mv_t mv)
{

    uint8_t buttons;
    waitms(200);
    mv_t mvolt = mv;
    lcd_clrscr();
    printf_P(ZmeritNapeti);
    do
    {
        buttons = RetButtons();

    }
    while(!(buttons&OK)&&!(buttons&ESC));

    if(buttons&OK)
    {


        // StartAD(TANK_CH);           /* prvni odmer se nepouziva */
        // ReadAD(TANK_CH);

        // StartAD(TANK_CH);
        /* TODO : by bylo lepsi prumerovat */
        // mvolt=RetTmV(ReadAD(TANK_CH));
        ClearTankMeas();
        lcd_clrscr();
        printf_P(Merim);

        do
        {

            //TankMeasPoll();
            ADCMeasPoll();

        }
        while(RetTankMeasMax()==0);

        mvolt = RetTankmV();


    }

    return mvolt;
}

/* Funkce pro zobrazeni hodnot na LCD */
uint8_t LcdRunTime=0;
uint8_t LcdRunTimeSet=0;
uint8_t LcdBlink=0;
uint8_t timeout=0;
uint8_t rst_timeout=0;
void LcdMenu(void)
{

    uint8_t buttons = RetButtons();
//    uint8_t print;

    static uint8_t viewState = 0;
    static uint8_t viewTime = 0;

    if(buttons) NoSleep = 1;
    else NoSleep = 0;


    if(CurrMenu<MINSMENU)
    {

        if(buttons & LEFT) DecMenu(MINMENU,MAXMENU);
        if(buttons & RIGHT) IncMenu(MINMENU,MAXMENU);

        if(buttons == OK && CurrMenu==1)    // kdyz drzi OK a zaroven je zobrazena trasa tak nulovat trasu po uplynuti timeoutu drzeni tlacitka
        {
            timeout++;
            rst_timeout=0;
        }
        else if(timeout)
        {

            rst_timeout++;

        }

        if(rst_timeout==20)     /* pokud se do cca 1s nezjisti ze je stale drzeno OK + ESC pak vymaz timeout.. */
        {

            rst_timeout=0;
            timeout=0;
        }

        if(timeout == CLEAR_TRIP_TIME)
        {
            ClearTripMeas();
            timeout=0;
        }

        /* print = CurrMenu*2;
         lcd_gotoxy(0,0);
         PrintVal(print++);
         lcd_gotoxy(0,1);
         PrintVal(print);*/
        lcd_gotoxy(0,0);

        fuel_t fcons;
        m_t meters;
        uint16_t mv = GetV12mV();
        if(mv%100 > 40) mv+=100;
        mv/=100;

        switch(CurrMenu)
        {

        case 0:
        {

            fuel_t acons = GetAvgFuelCons();
            if(acons > 999) acons = 999;
            if(GetCurrMeters() < 100 ) acons = 0;       /* zobrazi prumernou spotrebu az po ujeti 100m, do te doby nema moc vyznam..hodnoty vysoke kdyz auto vyjizdi */

            if(GetActMeters()>4)    /* jede vice jak 4ms/s ?? */
            {
                fcons = GetCurrFuelCons();
                if(fcons>999) fcons = 999;
                printf_P(PSTR("A:%2u,%01u / %2u,%01uL"),fcons/10,fcons%10,acons/10,acons%10);
            }
            else
            {
                fcons = GetCurrFuelConsL();
                if(fcons>999)fcons= 999;
                printf_P(PSTR("A:%2u,%01uLh %2u,%01uL"),fcons/10,fcons%10,acons/10,acons%10);
            }

            lcd_putc(CHAR_100KM_1);
            lcd_putc(CHAR_100KM_2);

            lcd_gotoxy(0,1);

            meters = GetCurrMeters()/100;

            printf_P(PSTR("%4lu,%01lu"),meters/10,meters%10);
            lcd_putc(CHAR_KM);

            if(getMenu1()==5) /* automaticky meneni */
            {

                if(CompWdTimer(viewTime,16))
                {

                    uint8_t sgt = SensorsGet();

                    viewState++;

                    if(viewState==0 && GetTankL() == 0)viewState++;         // uprava -- pokud nadrz je na nule nebo neni vubec inicializovana nemeri apod tak nezobrazuj distance.. nema to cenu zobrazovat


                    if(viewState==1 && (sgt != SENSOR_OUT_IN)) viewState++;
                    if(viewState==2 && (sgt == SENSOR_NO || sgt == SENSOR_OUT_IN)) viewState++;
                    if(viewState==3 && sgt != SENSOR_NO && sgt != SENSOR_OUT_IN) viewState++;

                    if(viewState>4)viewState=0;
                    viewTime = RetWdTimer();
                }
            }
            else if(getMenu1()<5)viewState=getMenu1();  /* pevna volba */
            else viewState=4;   // pruser

            switch(viewState)
            {

            case 0:
                printf_P(PSTR(" D: %4u"),GetMaxDis());
                lcd_gotoxy(15,1);
                lcd_putc(CHAR_KM);
                break;
            case 1:
                printf_P(PSTR("% 3i  % 3i"),RetSensorOut()/10,RetSensorIn()/10);
                lcd_putc(CHAR_STUPEN);
                lcd_gotoxy(10,1);
                lcd_putc(CHAR_IO1);
                lcd_putc(CHAR_IO2);
                break;
            case 2:
            {
                int temp;

                if(SensorsGet() == SENSOR_OUT) temp = RetSensorOut();
                else temp = RetSensorIn();

                if(getMenu1()==5){
                        temp/=10;
                        printf_P(PSTR("%2u,%01uV% 3i"),mv/10,mv%10,temp);
                }
                else printf_P(          PSTR(" % 3i,%01u  "),temp/10,(uint8_t)((temp<0?-temp:temp)%10));
                lcd_putc(CHAR_STUPEN);

            }
            break;
            case 3:
                printf_P(PSTR("%5u,%01u V"),mv/10,mv%10);
                break;
            case 4:
                fcons = GetCurrFuel();
                printf_P(PSTR("%5u,%01u L"), fcons/10,fcons%10);
                break;
            }





        }
        break;
        case 1:

            fcons = GetTripFuelCons();
            meters = GetTripMeters()/100;

            printf_P(PSTR("Trasa:%5u,%01uL"),fcons/10,fcons%10);
            lcd_putc(CHAR_100KM_1);
            lcd_putc(CHAR_100KM_2);
            lcd_gotoxy(0,1);

            fcons = GetTripFuel();

            printf_P(PSTR("%5lu,%01lu"),meters/10,meters%10);

            lcd_putc(CHAR_KM);

            printf_P(PSTR("%4u,%01u L"),fcons/10,fcons%10);

            break;
        case 2:

            printf_P(PSTR("Nadrz :%7u L\nDojezd:%7u "),GetTankL(),GetMaxDis());
            lcd_putc(CHAR_KM);
            //   lcd_gotoxy(1,0);
            //  lcd_putc(CHAR_A);
            //  lcd_gotoxy(4,0);
            //  lcd_putc(CHAR_Z);

            break;
        case 3:
            meters = GetActSpeed();
            printf_P(PSTR("Napeti:%5u,%01u V\nRychlost:%3lu,%01lu"),mv/10,mv%10,meters/10,meters%10);
            lcd_putc(CHAR_KM);
            lcd_putc('h');

            break;
        case 4:
            {
               int16_t out=RetSensorOut();

               int16_t in = RetSensorIn();

               uint8_t out1 = (out<0?-out:out)%10;
               uint8_t in1= (in<0?-in:in)%10;

            fcons = GetCurrFuel();
            printf_P(PSTR("A.Vyjeto:%3u,%01u L\n"), fcons/10,fcons%10);
            if(SensorsGet()==SENSOR_OUT_IN){


               printf_P(PSTR(" % 3i,%01u   % 3i,%01u "),out/10,out1,in/10,in1) ;
               lcd_putc(CHAR_STUPEN);
               lcd_gotoxy(7,1);
               lcd_putc(CHAR_STUPEN);
            }else{
                if(SensorsGet()==SENSOR_IN){
                        out=in;
                        out1=in1;
                }

                    printf_P(PSTR("Teplota: % 3i,%01u "),out/10,out1) ;
                    lcd_putc(CHAR_STUPEN);
            }
            }
            break;
        case 5:
            fcons = GetTotFuelCons();
            meters = GetTotMeters();
            meters/=1000;
            printf_P(PSTR("Celkem:%4u,%01uL\nNajeto: %7lu"),fcons/10,fcons%10,meters);
            lcd_putc(CHAR_KM);
            lcd_gotoxy(14,0);
            lcd_putc(CHAR_100KM_1);
            lcd_putc(CHAR_100KM_2);
            break;
        case 6:
            printf_P(Nastaveni);
            lcd_gotoxy(0,1);
            printf_P(EmptyLine);
            break;

        }

    }

    if(CurrMenu < MAXMENU)
    {

        /* if(IsRunning()){

            if( CompWdTimer(LcdRunTime,2) ) {

             LcdRunTime = RetWdTimer();

             LcdBlink = ~LcdBlink;

             }

         }else LcdBlink=0;




                 if(LcdBlink==0){

                     LcdBlinkView(':');


                 }else{

                     LcdBlinkView(' ');

                 }


        */
    }
    else        // kontrola prechodu do Nastaveni
    {

        if(CurrMenu==MAXMENU && (buttons&OK))
        {
            CurrMenu = MINSMENU;
            buttons &= ~OK;
        }
        if(CurrMenu>MAXMENU)
        {

            if(buttons&ESC) CurrMenu = MAXMENU;

            if(buttons&LEFT) DecMenu(MINSMENU,MAXSMENU);
            if(buttons&RIGHT) IncMenu(MINSMENU,MAXSMENU);


            // lcd_clrscr();
            lcd_gotoxy(0,0);
            uint8_t pos;
            switch(CurrMenu)
            {

            case EDIT_INJ:

                printf_P(KonstInj);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%3u,%01u ccm/min   "),GetCalibInjFlow()/10,GetCalibInjFlow()%10);
                if(buttons&OK)
                {
                    SetCalibInjFlow(EditCalibVal(GetCalibInjFlow(),4,1,50000,"cc/min"));
                    StoreCMeas();
                }

                break;
            case EDIT_VALVE:
                printf_P(KonstValve);

                printf(" %u\n",GetCalibValve());
                printf_P(EmptyLine);
                if(buttons&OK)
                {
                    SetCalibValve(EditCalibVal(GetCalibValve(),1,0,8,"vstrik."));
                    StoreCMeas();
                }
                break;
            case EDIT_IMP:

                printf_P(KonstImp);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%4u imp/100m   "),GetCalibMeters());
                if(buttons&OK)
                {
                    SetCalibMeters(EditCalibVal(GetCalibMeters(),5,0,50000,"imp/100m"));
                    StoreCMeas();
                }

                break;
            case EDIT_TANK_L:

                printf_P(TankObj);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%3u L           "),GetCalibL());
                if(buttons&OK)
                {
                    SetCalibL(EditCalibVal(GetCalibL(),3,0,500,"L"));
                    StoreCMeas();
                }

                break;
            case EDIT_TANK_MODE:
            {
                const char * p = text_tank_mode_standard;

                printf_P(TankMode);
                lcd_gotoxy(0,1);

                if(GetTankMode())
                {
                    p = text_tank_mode_int;
                }

                printf_P(p);

                if(buttons&OK)
                {

                    SetTankMode(!GetTankMode());

                }

            }
            break;
            case EDIT_TANK1:
            case EDIT_TANK2:
            case EDIT_TANK3:
            case EDIT_TANK4:
            case EDIT_TANK5:
                pos = CurrMenu-EDIT_TANK1;
                printf_P(TankKalib,pos+1);
                if(GetCalibIx(pos)==EE_INIT_B) printf_P(PSTR("\n%3u L,%4u mV"),GetCalibLx(pos),GetCalibVx(pos) );
                else printf_P(PSTR("\n  Nepouzivano  "));

                if(buttons&OK)
                {

                    lcd_clrscr();
                    printf_P(P("Pouzit pro nadrz\nESC=NE    OK=ANO"));

                    while(1)
                    {

                        uint8_t bt = RetButtons();

                        if(bt&OK)
                        {

                            SetCalibIx(pos,EE_INIT_B);
                            break;

                        }

                        if(bt&ESC)
                        {

                            SetCalibIx(pos,0);
                            return ;
                        }

                    }

                    mv_t mvolt=GetCalibVx(pos);

                    mvolt=LcdMeasmV(mvolt);


                    tank_t l = EditCalibVal(GetCalibLx(pos),3,0,500,"L");
                    if(pos>0 && l<=GetCalibLx(pos-1))
                    {

                        lcd_clrscr();
                        printf_P(P("Chyba hodnoty!\n%u. %u <= %u. %u"),pos,l,pos-1,GetCalibLx(pos-1));
                        waitms(3000);
                        return;

                    }
                    SetCalibVx(pos,EditCalibVal(mvolt,5,0,10000,"mV"));
                    SetCalibLx(pos,l);
                    StoreCMeas();
                }

                break;
                /*
                case EDIT_TANK1:

                            printf_P(Tank1Kalib);
                            lcd_gotoxy(0,1);
                            printf("%u L,%u mV",GetCalibL1(),GetCalibV1());

                            if(buttons&OK){

                                mv_t mvolt=GetCalibV1();

                                mvolt=LcdMeasmV(mvolt);


                                SetCalibV1(EditCalibVal(mvolt,5,0,10000,"mV")) ;

                                SetCalibL1(EditCalibVal(GetCalibL1(),3,0,500,"L"));




                            }
                            break;
                case EDIT_TANK2:

                            printf_P(Tank2Kalib);
                            lcd_gotoxy(0,1);
                            printf("%u L,%u mV",GetCalibL2(),GetCalibV2());

                            if(buttons&OK){
                                mv_t mvolt=GetCalibV2();

                                mvolt=LcdMeasmV(mvolt);


                                SetCalibV2(EditCalibVal(mvolt,5,0,10000,"mV")) ;

                                SetCalibL2(EditCalibVal(GetCalibL2(),3,0,500,"L"));


                            }
                            break;
                            */
            case EDIT_SENSORS:
            {
                const char * p = text_no_sensor;

                printf_P(EditSensors);
                lcd_gotoxy(0,1);

                switch(SensorsGet())
                {

                case SENSOR_NO:
                    p = text_no_sensor;
                    break;
                case SENSOR_IN:
                    p = text_in_sensor;
                    break;
                case SENSOR_OUT:
                    p = text_out_sensor;
                    break;
                case SENSOR_OUT_IN:
                    p = text_out_in_sensor;
                    break;
                }

                printf_P(p);

                if(buttons&OK)
                {

                    uint8_t set = SensorsGet();

                    set++;

                    if(set>SENSOR_OUT_IN)set = SENSOR_NO;

                    SensorsSet(set);

                }

            }
            break;
            case EDIT_SLEEP_TIME:
                printf_P(SleepTimeText);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%2u s            "),getSleepTime());
                if(buttons&OK)
                {
                    setSleepTime(EditCalibVal(getSleepTime(),2,0,60,"s"));

                }
                break;
            case EDIT_CALIB_VOLT:
                printf_P(CalibVoltText);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%3u,%01u kOhm      "),GetCalibVolt()/10,GetCalibVolt()%10);
                if(buttons&OK)
                {
                    SetCalibVolt(EditCalibVal(GetCalibVolt(),4,1,1100,"kOhm"));
                }
                break;
            case EDIT_MENU1:
                {
                const char * p = text_menu1_0;
                printf_P(Menu1Text);
                lcd_gotoxy(0,1);
                switch(getMenu1()){
                case 0:
                    p = text_menu1_0;
                    break;
                case 1:
                    p = text_menu1_1;
                    break;
                case 2:
                    p = text_menu1_2;
                    break;
                case 3:
                    p = text_menu1_3;
                    break;
                case 4:
                    p = text_menu1_4;
                    break;
                case 5:
                    p = text_menu1_5;
                    break;
                }
                printf_P(p);

                if(buttons&OK)
                {

                    uint8_t set = getMenu1();

                    set++;

                    if(set>5)set = 0;

                    setMenu1(set);

                }

                }
                break;
            case EDIT_RESET:
                printf_P(CelkReset);
                lcd_gotoxy(0,1);
                printf_P(EmptyLine);
                if(buttons&OK)
                {

                    printf_P(KomplReset);

                    do
                    {
                        buttons = RetButtons();

                    }
                    while(!(buttons&OK)&&!(buttons&ESC));

                    if(buttons&OK)
                    {

                        SetReboot();
                        ClearCMeas();
                        lcd_clrscr();
                        printf_P(Vymazano);
                        waitms(1000);

                    }


                }

                break;
            case EDIT_SHOW:
            {

                uint16_t meters = GetActSpeed();
                printf_P(PSTR("V:%03u,%02u P:%05u\nN:%4umV R:%3u,%01u"),GetInjCnt()/50,(GetInjCnt()%50)*2,GetImpCnt(),RetTankmV(),meters/10,meters%10);

            }

            break;
            default:
                break;

            }

        }
    }
}

/* hlavni smycka */

int main(void)
{

    uint8_t SleepTime=0;
    uint8_t Reboot=0;
    #ifdef USART_DEBUG
    char debug[200];
    #endif // USART_DEBUG
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;

    DDRD |= 1 << PD2;        /* lcd power on */
    PORTD &= ~(1 << PD2);

    waitms(400);

#ifdef USART_DEBUG
    uart_init(UART_BAUD_SELECT(115200,F_CPU));
    sprintf_P(debug,PSTR("DEBUG:Studeny start\n MCUSR=%u\nWDTCSR=%u\nDDRB=%u,PORTB=%u\nDDRC=%u,PORTC=%u\nDDRD=%u,PORTD=%u\n"),MCUSR,WDTCSR,DDRB,PORTB,DDRC,PORTC,DDRD,PORTD);
    uart_puts(debug);
#endif // USART_DEBUG


    disComparator();
    SetReboot();
    DisSleep();

#ifndef USART_DEBUG
    UCSR0A = 0;
    UCSR0B = 0;
#endif

    waitms(100);

#ifdef USART_DEBUG

    uart_puts_P("DEBUG:Hlavni smycka while(1)->\n");
#endif // USART_DEBUG
    while(1)
    {

        if(CheckReboot())        /* reset s wdt byl lehce problematicky takze se nakonec testuje reboot flag a probiha pak vlastne nova inicializace MCU po kazdem probuzeni z power down */
        {

#ifdef USART_DEBUG
     uart_puts_P("DEBUG:CheckReboot() TRUE\n");
#endif // USART_DEBUG
            PRR = 0;

            DDRD |= 1 << PD2;        /* lcd power on */
            PORTD &= ~(1 << PD2);

            waitms(200);

            lcd_init(LCD_DISP_ON); // init LCD

            lcd_clrscr();

         //   waitms(100);

            FILE lcd_stream = FDEV_SETUP_STREAM(lcd_putc_stream, NULL, _FDEV_SETUP_WRITE);

            stdout = &lcd_stream;

            lcd_cgram_write(char_stupen,CHAR_STUPEN);           /* ukladani uzivatelskych znaku do LCD */
            lcd_cgram_write(char_100km_1,CHAR_100KM_1);
            lcd_cgram_write(char_100km_2,CHAR_100KM_2);
            lcd_cgram_write(char_km,CHAR_KM);
            lcd_cgram_write(char_a,CHAR_A);
            lcd_cgram_write(char_z,CHAR_Z);
            lcd_cgram_write(char_io1,CHAR_IO1);
            lcd_cgram_write(char_io2,CHAR_IO2);

         //   waitms(200);

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), lcd_init, cgram_write complet. \n");
#endif // USART_DEBUG

            InitWdTimer();
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), InitWdTimer() \n");
#endif // USART_DEBUG
            InitCMeas();
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), InitCMeas() \n");
#endif // USART_DEBUG
            InitButtons();
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), InitButtons() \n");
#endif // USART_DEBUG
#ifndef USART_DEBUG
            SensorInit();
#endif
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), SensorInit() zastaven, debug mod vyzaduje USART piny!! \n");
#endif // USART_DEBUG

            lcd_clrscr();

            printf_P(LcdStart1);        /* tiskne startovaci napis a verzi */
            lcd_gotoxy(0,1);
            printf_P(LcdStartVer);
            lcd_gotoxy(2,1);

            printf_P(PSTR("0.3.3.17"));

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), LCD bezi, zobrazeni verze, cekam 2s na pokracovani\n");
#endif // USART_DEBUG

            waitms(2000);

            lcd_clrscr();

            Reboot = 1;
        }

        if(Reboot && IsRunning())   /* pokud je to prvni start po reboot tak ukazuj prumernou spotrebu a akt trasu tak dlouho dokud nenastartuje motor */
        {
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:Reboot && IsRunning() \n");
#endif // USART_DEBUG
            ClearAvgMeas();
            Reboot = 0;
        }
#ifndef USART_DEBUG
        SensorsPoll();      /* ds18b20 odmery*/
#endif

#ifdef USART_DEBUG
    uart_puts("DEBUG:Run..\n");
#endif // USART_DEBUG

        //TankMeasPoll();     /* nadrz odmery*/
        ADCMeasPoll();
        //V12MeasPoll();      /* napeti odmery*/
        LcdMenu();          /* LCD menu */

        waitms(20);        // zde musi byt nejaka prodleva kvuli zablikavani LCD, ktere je videt na blikajici dvojtecce a pak v v MENU nastaveni - setup
        // doba po kterou je stabilni aktualni obraz

        if(!IsRunning() && !NoSleep )
        {

            if(!SleepTimeout)
            {
                SleepTime = RetWdTimer();
                SleepTimeout = 1;
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:SleepTimeout=1\n");
#endif // USART_DEBUG
            }

        }
        else SleepTimeout=0;

        if(SleepTimeout && CompWdTimer(SleepTime,4*getSleepTime()))
        {

            SleepTimeout = 0;
            SetSleep() ;
#ifdef USART_DEBUG
    uart_puts_P("DEBUG:SetSleep()\n");
#endif // USART_DEBUG

        }

        SleepPoll();        // test zda prejit do sleep


    }





}
