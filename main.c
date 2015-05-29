/****************************************************************************
*
* Authors:    Ondrej Lutera
* Modifications: Lukas Soukup
* Copyright:  GPL
*
*
*****************************************************************************/
#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>

//#define USART_DEBUG - moved to global.h

#include "global.h"
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
#include <math.h>

#ifdef USART_DEBUG
#include "uart.h"
char debug[200];
#endif // USART_DEBUG

/* definice MAX a MIN pro SETUP MENU a jednotlive poradi a cislovani MENU - je pouzita spolecna promenna proto je setup menu s offsetem +100 */

//keep these strings aligned to 16 chars
const char PROGMEM FirmwareDescr[]= "PP_Light2 ZX ed.";
const char PROGMEM FirmwareVer[] =  "0.6.2d@0.3.3.17c";

#define MAIN_LOOP_SPEED 100 //delay in ms
#define SPACES_DIST  5
#define SPACES_SPEED 3
#define SPACES_SPEED_AVG 4
#define SPACES_CONS  4
#define SPACES_TEMP  2
#define SPACES_OTHER 4
#define SPACES_RANGE 3
#define SPACES_REM_HRS 3

#define EDIT_INJ  200       // editace konstanty vstrikovace v ccm/min
#define EDIT_CYLS 201       // editace poctu valcu motoru
#define EDIT_IMP  202       // editace poctu impulzu na 100m
#define EDIT_SPEED_RATIO 203
#define EDIT_TANK_C 204     // editace objemu nadrze
#define EDIT_TANK_MODE 205
#define EDIT_TANK_FUEL_SRC 206
#define EDIT_TANK1 207      // prvni kalib. konstanty napeti v mV a litry nadrze   -- nadrz pocita pres linearni fci -- sestupna ci rostouci dle konstant
#define EDIT_TANK2 208      // druha skupina kalib. konstanty napeti v mV a litry nadrze
#define EDIT_TANK3 209
#define EDIT_TANK4 210
#define EDIT_TANK5 211
#define EDIT_SENSORS 212
#define EDIT_SLEEP_TIME 213
#define EDIT_CALIB_VOLT 214
#define EDIT_SUBDISP  215
#define EDIT_SUBDISP_CHANGE_SPEED 216
#define EDIT_SEL_DISP_WIDTH 217
#define EDIT_RESET 218     // kompletni reset vymazani
#define EDIT_SHOW_RAW  219
#define EDIT_VERSION_INFO 220

#define MINSMENU EDIT_INJ
#define MAXSMENU EDIT_VERSION_INFO

/* Definice MENU pro zobrazeni provoznich hodnot. Zde menit poradi editaci cisel ! */

#define DEF_SUBDISPLAYS 8

/* definice pořadí položek menu*/
#define DISP_DEFAULT             0
#define DISP_TRIP                1
#define DISP_TANK_TRIP           2
#define DISP_TOTALS              3
#define DISP_SETTINGS            4

// effectively ommited
#define DISP_TANK_RANGE          5
#define DISP_INFO_VOLTAGE_SPEED  6
#define DISP_CURR_FUEL_TEMP      7

#define MINMENU DISP_DEFAULT
#define MAXMENU DISP_SETTINGS

/*
#define AKTUSP      0           // aktualni spotreba        xx,x l/100km ci xx,x l/h pokud vuz stoji
#define PRUMSP      1           // prumerna spotreba na danou aktualni jizdu    xx,x l/100km

#define TRASSP      2           // trasa spotreba - mazatelna az uzivatelem     xx,x l/100km
#define DOJEZD      3           // dojezd pocitany z prumerne spotreby a stavu nadrze   xxx km

#define TRAVYJ      4           // trasa vyjete litry benzinu       xx L
#define AKTVYJ      5           // aktualne vyjete litry benzinu po aktualni jizdu xx L

#define AKTUUJ      6           // aktualne ujeta vzdalenost ve formatu xxxx,x km
#define TRASUJ      7           // trasa ujeta vzdalenost format xxxx,x km

#define TEPVEN      8           // teplota venkovni cidlo format xx,x °C
#define TEPVNI      9           // teplota vnitrni cidlo format xx,x °C

#define NAPETI      10          // napeti baterie - format xx,x V
#define NADRZ       11          // stav nadrze format xx L

#define CELKUJ      12          // celkove ujete km ve formatu xxxxxxxxx km - mazatelne pouze kompletnim resetem
#define CELKSP      13          // celkova spotreba ve fornatu xx,x l/100km
*/

#define SLEEP_TIME 120 /* 30s = 4*30 */

#define RESETABLES__CLEAR_TIMEOUT 4 /* 1s = 4*1 */
#define SPEED_KMPH_LPH_THERSHOLD 10 // 10km/h

/* text popisky ulozene ve flash */

const char PROGMEM Nastaveni[] ="    Nastaveni                           ";
const char PROGMEM EmptyLine[] ="                                        ";

const char PROGMEM KonstInj[] = "Vstrik.konstanta";
const char PROGMEM KonstImp[] = "Metry  konstanta";
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
const char PROGMEM text_no_sensor[] =     "Zadne cidlo     ";
const char PROGMEM text_out_sensor[] =    "Cidlo c.1       ";
const char PROGMEM text_in_sensor[] =     "Cidlo c.2       ";
const char PROGMEM text_out_in_sensor[] = "Obe cidla       ";
const char PROGMEM TankMode[] = "Rezim nadrze:   ";
const char PROGMEM text_tank_mode_standard[] = "  standardni    ";
const char PROGMEM text_tank_mode_int[] = "  detekce imp.  ";
const char PROGMEM TankModeFuelSrc[] = "Mnoz. paliva:   ";
const char PROGMEM text_tank_mode_src_tank[] = "  mereni        ";
const char PROGMEM text_tank_mode_src_calc[] = "  vypocet       ";
const char PROGMEM SleepTimeText[] = "Prodleva vypnuti";
const char PROGMEM CalibVoltText[] = "Kalib. nap. site";
const char PROGMEM DefSubDispText[]=      "Volba zobrazeni ";
const char PROGMEM text_defsubdisp_0[]=   "  Automaticky   ";
const char PROGMEM text_defsubdisp_1[]=   "     Dojezd     ";
const char PROGMEM text_defsubdisp_2[]=   "   Obe teploty  ";
const char PROGMEM text_defsubdisp_3[]=   "    Teplota     ";
const char PROGMEM text_defsubdisp_4[]=   "     Napeti     ";
const char PROGMEM text_defsubdisp_5[]=   "  Vyjete litry  ";
const char PROGMEM text_defsubdisp_6[]=   "Palivo v nadrzi ";
const char PROGMEM text_defsubdisp_7[]=   "Aktual. rychlost";
const char PROGMEM text_defsubdisp_8[]=   "Prumer. rychlost";
const char PROGMEM SubDispChangeSpeed[]=  "Rychlost zobr.  ";
const char PROGMEM text_speed_ratio[] =   "Pomer rychlosti ";
const char PROGMEM SubDispDisplayWidth[]= "Sirka displeje  ";

/* uzivatelske znaky v CGRAM lcd */

#define CHAR_STUPEN  0
#define CHAR_100KM_1 1
#define CHAR_100KM_2 2
#define CHAR_KM      3
#define CHAR_Z       4
#define CHAR_A       5
#define CHAR_IO1     6
#define CHAR_IO2     7

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
    0b00010101,
    0b00010110,
    0b00010101,
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
uint16_t EditCalibVal(uint16_t val,uint8_t cnt,uint8_t prec,uint16_t max,char *text);
void LcdBlinkView(char c);
mv_t LcdDatamV(mv_t mv);

// value display helper methods
void PrintFuelCons(fuel_t f);
void PrintFuelConsL(fuel_t f);
void PrintDist(dist_km_t m);
void PrintRange(dist_km_t m);
void PrintFuel(fuel_t f);
void PrintFuelShort(fuel_t f);
void PrintSpeed(speed_t s);
void PrintSpeedAvg(speed_t s);
void PrintVoltage(v_t v);
void PrintTemperature(t_t t);
void PrintRemHrs(h_t h);
// end of value display helper methods
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
mv_t LcdDatamV(mv_t mv)
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
        ClearTankMeas();
        lcd_clrscr();
        printf_P(Merim);

        do
        {
            ADCMeasPoll();
        }
        while(RetTankMeasMax()==0);

        mvolt = RetTankmV();
    }
    return mvolt;
}

uint64_t MyPow(uint16_t base, uint16_t exp) {
   uint64_t ret = 1;
   while (exp > 0) {
	   ret *= base;
	   exp--;
   }
   return ret;
}

uint16_t NeededPlaces(uint64_t val) {
	uint64_t workVal = val;
	uint16_t ret = 1;
	while (workVal >= 10) { 
		ret++;
		workVal /= 10;
	}
	return ret;	
}

void PrintDecimalValue(float val, uint8_t displaySpaces) { //uint8_t displayDecimals
#ifdef USART_DEBUG
char msg[100];
#endif
    if (isnan(val) || val < 0)
	{
		#ifdef USART_DEBUG
		sprintf_P(msg, PSTR("Value NaN or negative - not supported\n"));
		uart_puts(msg);
		#endif
		for(int8_t i = 0; i< displaySpaces; i++) {
			lcd_putc('-');
		}
		return;
	}
	//int negative = 0;
	//if (val < 0)
	//{
		//val*= -1;
		//negative = 1;
	//}
#ifdef USART_DEBUG
sprintf_P(msg, PSTR("PrintDecimalValue() val*1000=%u; val=%u,%u; displaySpaces=%u;\n"), (uint32_t)floor(val*1000), (uint32_t)floor(val), (uint32_t)((val-floor(val))*1000000), displaySpaces);
uart_puts(msg);
#endif
    // celá část
	uint32_t M = (uint32_t)floor(val);
	int16_t Mp = NeededPlaces(M);
#ifdef USART_DEBUG
sprintf_P(msg, PSTR("M=%lu; Mp=%i;\n"), M, Mp);
uart_puts(msg);
#endif
	if (Mp > displaySpaces) {
#ifdef USART_DEBUG
sprintf_P(msg, PSTR("Value out of range\n"), M, Mp);
uart_puts(msg);
#endif
		for(int8_t i = 0; i< displaySpaces; i++) {
			lcd_putc('#');
		}
		return;
	}


	int16_t mp = displaySpaces-1-Mp;
	if (mp < 0) mp = 0;
	uint16_t multiplier = (uint16_t)MyPow(10, mp);
	uint32_t m = (uint32_t)floor((val - M)*multiplier);

	#ifdef USART_DEBUG
	sprintf_P(msg, PSTR("m=%lu; mp=%i; multiplier=%u; \n"), m, mp, multiplier);
	uart_puts(msg);
	#endif
	
	char fmt[20];
	if (mp == 0) // nejsou desetiny
	{ 
		sprintf_P(fmt, PSTR("%%%ulu"), displaySpaces);
	}
	else 
	{
		sprintf_P(fmt, PSTR("%%%ulu,%%0%ulu"), Mp, mp);
	}
	#ifdef USART_DEBUG
	sprintf_P(msg, PSTR("fmt=%s\n"), fmt);
	uart_puts(msg);
	#endif

	printf(fmt,M,m);
}

void PrintFuelCons(fuel_t f) {
	PrintDecimalValue(f, SPACES_CONS);
    lcd_putc(CHAR_100KM_1);
    lcd_putc(CHAR_100KM_2);
}

void PrintFuelConsL(fuel_t f) {
	PrintDecimalValue(f, SPACES_CONS);
    printf_P(PSTR("Lh"));
}


void PrintFuel(fuel_t f) {
	PrintDecimalValue(f, SPACES_CONS);
	printf_P(PSTR("L"));
}

void PrintFuelShort(fuel_t f) {
	PrintDecimalValue(f, 2);
	printf_P(PSTR("L"));
}

void PrintDist(dist_km_t km) {
    PrintDecimalValue(km, SPACES_DIST);
    lcd_putc(CHAR_KM);
}

void PrintRange(dist_km_t km) {
	PrintDecimalValue(km, SPACES_RANGE);
	lcd_putc(CHAR_KM);
}

void PrintSpeed(speed_t s) {
	PrintDecimalValue(s, SPACES_SPEED);
	lcd_putc(CHAR_KM);
	lcd_putc('h');
}

void PrintSpeedAvg(speed_t s) {
	PrintDecimalValue(s, SPACES_SPEED_AVG);
	lcd_putc(CHAR_KM);
	lcd_putc('h');
}

void PrintVoltage(v_t v) {
	PrintDecimalValue(v, SPACES_OTHER);
	lcd_putc('V');
}

void PrintTemperature(t_t t) {
	char sign = ' ';
	int spaceReduction = 0;
	if (t < 0) {
		sign = '-';
		t = -t;
	}
	if (t < 10) {
	  lcd_putc(' ');
	  spaceReduction = 1;
	}
	lcd_putc(sign);
	PrintDecimalValue(t, SPACES_TEMP-spaceReduction);
	lcd_putc(CHAR_STUPEN);
}

void PrintRemHrs(h_t t) {
	PrintDecimalValue(t, SPACES_REM_HRS);
	lcd_putc('h');
}

uint8_t IsMoving() {
  return GetInstSpeed()>SPEED_KMPH_LPH_THERSHOLD;
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

    static uint8_t viewState = 1; // 0 is reserved for AutoChange
    static uint8_t viewTime = 0;

    if(buttons) NoSleep = 1;
    else NoSleep = 0;

    if(CurrMenu<MINSMENU)
    {

        if(buttons & LEFT) DecMenu(MINMENU,MAXMENU);
        if(buttons & RIGHT) IncMenu(MINMENU,MAXMENU);

        if(buttons == OK && (CurrMenu==DISP_DEFAULT || CurrMenu==DISP_TRIP  || CurrMenu==DISP_TANK_TRIP))    // kdyz drzi OK a zaroven je zobrazena trasa nebo nadrz tak nulovat trasu(nadrz) po uplynuti timeoutu drzeni tlacitka
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

        if(timeout == RESETABLES__CLEAR_TIMEOUT)
        {
            switch(CurrMenu) {
			  case DISP_DEFAULT :   ClearCurrData(); break;
		      case DISP_TRIP :      ClearTripData(); break;
			  case DISP_TANK_TRIP : ClearTankData(); break;
			  default: break;
			}
			lcd_clrscr();
			waitms(500);
            timeout=0;
        }

        lcd_gotoxy(0,0);

        fuel_t fcons = 0;

		char runIndicator = IsRunning() ? '*' : '#';
        switch(CurrMenu)
        {

        case DISP_DEFAULT: 
        {
			lcd_putc(runIndicator);
            if (!IsWide()) {
				fuel_t ccons = GetCurrFuelCons();

				if(GetInstSpeed()>SPEED_KMPH_LPH_THERSHOLD)    /* jede vice jak x ms/s ?? */
				{
					fcons = GetInstFuelCons();
					PrintDecimalValue(fcons, SPACES_CONS); printf_P(PSTR(" / ")); PrintDecimalValue(ccons, SPACES_CONS);
				}
				else
				{
					fcons = GetInstFuelConsL();
					PrintDecimalValue(fcons, SPACES_CONS); printf_P(PSTR("Lh ")); PrintDecimalValue(ccons, SPACES_CONS);
				}
				lcd_putc(CHAR_100KM_1);
				lcd_putc(CHAR_100KM_2);

				lcd_gotoxy(0,1);
			}
			else { // WIDE
				lcd_putc(' ');
				if(IsMoving())    /* jede vice jak x ms/s ?? */
				{
					PrintFuelCons(GetInstFuelCons());
				}
				else
				{
					PrintFuelConsL(GetInstFuelConsL());
				}
				printf_P(PSTR(" "));
				PrintDist(GetCurrKMeters());
				printf_P(PSTR("  "));
				PrintFuel(GetCurrFuel());
				printf_P(PSTR("  "));
				PrintSpeedAvg(GetCurrSpeed());
				printf_P(PSTR("  "));
				PrintFuelCons(GetCurrFuelCons());
				fcons = GetCurrFuelCons();
			}

            if (!IsWide()) {
				PrintDist(GetCurrKMeters()); lcd_putc(' ');
			
				lcd_gotoxy(7,1); // fixed start position of subdisplay

				if (GetDefSubDisp() > DEF_SUBDISPLAYS) { SetDefSubDisp(0); } // safe-check
			
				if (GetDefSubDisp() > 0) { viewState = GetDefSubDisp(); } /* pevna volba */
				else /* GetDefSubDisp()==0 - automaticke meneni */
				{

					if(CompWdTimer(viewTime,GetSubDispChangeSpeed()))
					{
						viewState++;
						uint8_t sgt = SensorsGet();

						// vynechavat nesmyslne hodnoty
						if(viewState==1 && GetTankL() == 0) viewState++;         // uprava -- pokud nadrz je na nule nebo neni vubec inicializovana nemeri apod tak nezobrazuj distance.. nema to cenu zobrazovat
						if(viewState==2 && (sgt != SENSOR_OUT_IN)) viewState++;
						if(viewState==3 && (sgt == SENSOR_NO || sgt == SENSOR_OUT_IN)) viewState++;
						if(viewState==4 && (sgt != SENSOR_NO && sgt != SENSOR_OUT_IN)) viewState++;

						if (viewState > DEF_SUBDISPLAYS) viewState=1;                    
					
						viewTime = RetWdTimer();
					}
				}

				switch(viewState)
				{

				case 1:
					printf_P(PSTR(" D  "),GetCurrRange());
					PrintRange(GetCurrRange());
					break;
				case 2:
					printf_P(PSTR("% 3i  % 3i"),RetSensorOut()/10,RetSensorIn()/10);
					lcd_putc(CHAR_STUPEN);
					lcd_gotoxy(10,1);
					lcd_putc(CHAR_IO1);
					lcd_putc(CHAR_IO2);
					break;
				case 3:
				{
					int temp;

					if(SensorsGet() == SENSOR_OUT) temp = RetSensorOut();
					else temp = RetSensorIn();

					if(GetDefSubDisp()==0){
							temp/=10;
							PrintDecimalValue(GetV12V(), 4);
							printf_P(PSTR("V% 3i"),temp);
					}
					else printf_P(PSTR(" % 3i,%01u  "),temp/10,(uint8_t)((temp<0?-temp:temp)%10));
					lcd_putc(CHAR_STUPEN);
				}
				break;
				case 4:
					printf_P(PSTR("  "));
					PrintDecimalValue(GetV12V(), 5);
					printf_P(PSTR("V"));
					break;
				case 5:
					printf_P(PSTR("    "));
					PrintFuel(GetCurrFuel());
					break;
				case 6: {
					//tank_t tankL = GetTankL();
					//printf_P(PSTR(" N%3u,%01u L"), tankL/10,tankL%10);
					printf_P(PSTR(" N  "));
					PrintFuel(GetTankL()); 
					}
					break;
				case 7:
					//speed = GetInstSpeed(); // in km/h with 1 dec. presicion
					//printf_P(PSTR(" R%3u,%01u"), speed/10,speed%10);
					//lcd_putc(CHAR_KM);
					//lcd_putc('h');
					printf_P(PSTR(" R"));
					PrintSpeed(GetInstSpeed());
					break;
				case 8:
					printf_P(PSTR(" P"));
					PrintSpeedAvg(GetCurrSpeed());
					break;
				}
			}
        }
        break;
        case DISP_TRIP:
            printf_P(IsWide() ? PSTR("Trasa :  ") : PSTR("Tr:"));
            PrintDist(GetTripKMeters());
            printf_P(PSTR("  "));
			PrintFuel(GetTripFuel());
			if (!IsWide()) {
			  lcd_gotoxy(0,1);
			}
            printf_P(PSTR("  "));
			PrintSpeedAvg(GetTripSpeed());
            printf_P(PSTR("  "));
			fcons = GetTripFuelCons();
			PrintFuelCons(fcons);
            break;
        case DISP_TANK_TRIP:
            printf_P(IsWide() ? PSTR("Nadrz :  ") : PSTR("Nd:"));
            PrintDist(GetTankKMeters());
            printf_P(PSTR("  "));
            PrintFuel(GetTankFuel());
			if (!IsWide()) {
            lcd_gotoxy(0,1);
			}
            printf_P(PSTR("  "));
            PrintSpeedAvg(GetTankSpeed());
            printf_P(PSTR("  "));
			fcons = GetTankFuelCons();
            PrintFuelCons(fcons);
        break;
        case DISP_TANK_RANGE:
            printf_P(PSTR("Nadrz :   "));
			PrintFuel(GetTankL());
			printf_P(PSTR("\nDojezd:   "));
			PrintDist(GetCurrRange());
            break;
        case DISP_INFO_VOLTAGE_SPEED:
            printf_P(PSTR("Napeti:   "));
			PrintDecimalValue(GetV12V(), 5);
            printf_P(PSTR("V\nRychlost:"));
			PrintSpeed(GetInstSpeed());

            break;
        case DISP_CURR_FUEL_TEMP:
            {
               int16_t out=RetSensorOut();

               int16_t in = RetSensorIn();

               uint8_t out1 = (out<0?-out:out)%10;
               uint8_t in1= (in<0?-in:in)%10;

            printf_P(PSTR("A.Vyjeto: ")); PrintFuel(GetCurrFuel());
			lcd_gotoxy(0,1);
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
        case DISP_TOTALS:
            printf_P(IsWide() ? PSTR("Celkem:  ") : PSTR("Ck:"));
            PrintDist(GetTotKMeters());
            printf_P(PSTR("  "));
            PrintFuel(GetTotFuel());
            if (!IsWide()) {
            lcd_gotoxy(0,1);
            }
            printf_P(PSTR("  "));
            PrintSpeedAvg(GetTotSpeed());
            printf_P(PSTR("  "));
            PrintFuelCons(GetTotFuelCons());
			fcons = GetTotFuelCons();
            break;
        case DISP_SETTINGS:
            printf_P(Nastaveni);
            lcd_gotoxy(0,1);
            printf_P(EmptyLine);
            break;
        }
		if (IsWide()) {
			if (CurrMenu != DISP_SETTINGS) {
				lcd_gotoxy(0,1);
				printf_P(PSTR("R "));
				PrintSpeed(GetInstSpeed());
				printf_P(PSTR("  D "));
				//PrintRange(GetRange(GetTotFuelCons()));
				//PrintRange(GetRange(fcons));
				PrintDecimalValue(GetRange(GetTotFuelCons()), SPACES_RANGE);
				lcd_putc('/');
				PrintDecimalValue(GetRange(fcons), SPACES_RANGE);
				lcd_putc(CHAR_KM);
				if (CurrMenu == DISP_DEFAULT) {
					lcd_putc(' ');
					if (IsMoving()) {
						PrintRange(GetRange(GetInstFuelCons()));
					}
					else {
						PrintRemHrs(GetInstRemHrs());
					}
				}
				else {
					for(uint8_t i = 0; i < SPACES_CONS+3; i++) {
						lcd_putc(' ');
					}
				}
				lcd_gotoxy(26,1);
				//if (GetTankFuelSrc()) { lcd_putc('#'); } else { lcd_putc('*'); }
				PrintDecimalValue(GetTankL(),3);
				lcd_putc('L');
				lcd_putc(' ');
				PrintVoltage(GetV12V());
				PrintTemperature((t_t)(RetSensorOut())); // temperature is being returned with one decimal point precision
			}
		}
    }

    if(CurrMenu >= MAXMENU)
    {

        if(CurrMenu==MAXMENU && (buttons&OK))
        {
            CurrMenu = MINSMENU;
            buttons &= ~OK;
        }
        if(CurrMenu>MAXMENU)
        {

            if(buttons&ESC) {
				CurrMenu = MAXMENU;
				StoreBCData();
				StoreCarData();
				}

            if(buttons&LEFT) DecMenu(MINSMENU,MAXSMENU);
            if(buttons&RIGHT) IncMenu(MINSMENU,MAXSMENU);

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
                    //StoreCarData();
                }

                break;
            case EDIT_CYLS:
                printf_P(KonstValve);

                printf(" %u\n",GetCalibCyls());
                printf_P(EmptyLine);
                if(buttons&OK)
                {
                    SetCalibCyls(EditCalibVal(GetCalibCyls(),1,0,8,"vstrik."));
                    //StoreCarData();
                }
                break;
            case EDIT_IMP:
                printf_P(KonstImp);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%4u imp/100m   "),GetCalibMeters());
                if(buttons&OK)
                {
                    SetCalibMeters(EditCalibVal(GetCalibMeters(),5,0,50000," imp/100m"));
					//StoreCarData();
                }
                break;
			case EDIT_SPEED_RATIO:
				printf_P(text_speed_ratio);
				lcd_gotoxy(0,1);
                printf_P(PSTR("%3u %% z vzd.    "),GetCalibSpeedRatio());
                if(buttons&OK)
                {
	                SetCalibSpeedRatio(EditCalibVal(GetCalibSpeedRatio(),5,0,200," %"));
					//StoreCarData();
                }
				break;	
            case EDIT_TANK_C:
                printf_P(TankObj);
                lcd_gotoxy(0,1);
                printf_P(PSTR("%3u L           "),GetCalibTankL());
                if(buttons&OK)
                {
                    SetCalibTankL(EditCalibVal(GetCalibTankL(),3,0,500,"L"));
                    //StoreCarData();
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
            case EDIT_TANK_FUEL_SRC:
            {
	            const char * p = text_tank_mode_src_tank;

	            printf_P(TankModeFuelSrc);
	            lcd_gotoxy(0,1);

	            if(GetTankFuelSrc())
	            {
		            p = text_tank_mode_src_calc;
	            }
	            printf_P(p);
	            if(buttons&OK)
	            {
		            SetTankFuelSrc(!GetTankFuelSrc());
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

                    mvolt=LcdDatamV(mvolt);


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
                    //StoreCarData();
                }

                break;
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
            case EDIT_SUBDISP:
                {
                const char * p = text_defsubdisp_0;
                printf_P(DefSubDispText);
                lcd_gotoxy(0,1);
                switch(GetDefSubDisp()){
                case 0:
                    p = text_defsubdisp_0;
                    break;
                case 1:
                    p = text_defsubdisp_1;
                    break;
                case 2:
                    p = text_defsubdisp_2;
                    break;
                case 3:
                    p = text_defsubdisp_3;
                    break;
                case 4:
                    p = text_defsubdisp_4;
                    break;
                case 5:
                    p = text_defsubdisp_5;
                    break;
                case 6:
					p = text_defsubdisp_6;
					break;
				case 7:
				    p = text_defsubdisp_7;
					break;
				case 8:
					p = text_defsubdisp_8;
					break;		
                }
                printf_P(p);

                if(buttons&OK)
                {
                    uint8_t set = GetDefSubDisp();
                    if(++set>DEF_SUBDISPLAYS)set = 0;
                    SetDefSubDisp(set);
                }

                }
                break;
            case EDIT_SUBDISP_CHANGE_SPEED:
				printf_P(SubDispChangeSpeed);
				lcd_gotoxy(0,1);
				printf_P(PSTR("%2u 1/4s         "),GetSubDispChangeSpeed());
				if(buttons&OK)
				{
					SetSubDispChangeSpeed(EditCalibVal(GetSubDispChangeSpeed(),2,0,20,"1/4s"));
				}
            break;
            case EDIT_SEL_DISP_WIDTH:
				printf_P(SubDispDisplayWidth);
				lcd_gotoxy(0,1);
				printf_P(GetWideMode() ? PSTR("   40 znaku     ") : PSTR("   16 znaku     "));
				if(buttons&OK)
				{
					SetWideMode(!GetWideMode());
					RefreshWideFlag();
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
                        ClearData();
                        lcd_clrscr();
                        printf_P(Vymazano);
                        waitms(1000);

                    }


                }

                break;
            case EDIT_SHOW_RAW: 
			    {
				lcd_gotoxy(0,0);
				printf_P(P("V:"));
				PrintDecimalValue(GetInjCnt(), 5);
				lcd_putc(' ');
				printf_P(P("P:"));
				PrintDecimalValue(GetImpCnt(), 5);
				lcd_putc(' ');
				if (IsWide())
				{
					printf_P(P("B:"));
					PrintDecimalValue(GetCurrFuel(), 6);
				}
				lcd_gotoxy(0,1);
				printf_P(P("N:"));
				PrintDecimalValue(RetTankmV(), 5);
				lcd_putc(' ');
				printf_P(P("R:"));
				PrintDecimalValue(GetInstSpeed(), 5);
				lcd_putc(' ');
                //uint16_t meters = GetInstSpeed();
                //printf_P(PSTR("V:%03u,%02u P:%05u\nN:%4umV R:%3u,%01u"),GetInjCnt()/50,(GetInjCnt()%50)*2,GetImpCnt(),RetTankmV(),meters/10,meters%10);
				}
                break;
			case EDIT_VERSION_INFO:
				printf_P(FirmwareDescr);        /* tiskne startovaci napis a verzi */
				lcd_gotoxy(0,1);
				printf_P(FirmwareVer);
                break;
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
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;

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
            PRR = 0; // vypnutí všech úsporných opatření

#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckPoint A\n");
#endif // USART_DEBUG

            lcd_init(LCD_DISP_ON); // init LCD, včetně zapnutí

#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckPoint B\n");
#endif // USART_DEBUG

            // vložení uživatelských znaků do LDC
            lcd_cgram_write(char_stupen,CHAR_STUPEN);
            lcd_cgram_write(char_100km_1,CHAR_100KM_1);
            lcd_cgram_write(char_100km_2,CHAR_100KM_2);
            lcd_cgram_write(char_km,CHAR_KM);
            lcd_cgram_write(char_a,CHAR_A);
            lcd_cgram_write(char_z,CHAR_Z);
            lcd_cgram_write(char_io1,CHAR_IO1);
            lcd_cgram_write(char_io2,CHAR_IO2);

#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckPoint E\n");
#endif // USART_DEBUG

            lcd_waitbusy(); // počkat na dokončení zpracování poslední provedené operace
			FILE lcd_stream = FDEV_SETUP_STREAM(lcd_putc_stream, NULL, _FDEV_SETUP_WRITE);

#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckPoint C\n");
#endif // USART_DEBUG

			stdout = &lcd_stream;
			
#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckPoint D\n");
#endif // USART_DEBUG

            InitWdTimer();

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), InitWdTimer() \n");
#endif // USART_DEBUG

            InitDataAndApp();

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:CheckReboot(), InitDataAndApp() \n");
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

#ifdef USART_DEBUG
uart_puts_P("DEBUG:CheckReboot(), LCD bezi\n");
#endif // USART_DEBUG

            Reboot = 1;
        } //end of init after reboot

        if(Reboot && IsRunning())   /* pokud je to prvni start po reboot tak ukazuj prumernou spotrebu a akt trasu tak dlouho dokud nenastartuje motor */
        {

#ifdef USART_DEBUG
    uart_puts_P("DEBUG:Reboot && IsRunning() \n");
#endif // USART_DEBUG

            ClearCurrData();
            Reboot = 0;
        }


#ifndef USART_DEBUG
        SensorsPoll();      /* ds18b20 odmery*/
#endif

#ifdef USART_DEBUG
    uart_puts("#");
#endif // USART_DEBUG

        ADCMeasPoll();
        LcdMenu();          /* LCD menu */

        waitms(MAIN_LOOP_SPEED); // zde musi byt nejaka prodleva kvuli zablikavani LCD, ktere je videt na blikajici dvojtecce a pak v MENU nastaveni - setup (doba po kterou je stabilni aktualni obraz)

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
            SetSleep();
        }

        SleepPoll();        // test zda prejit do sleep
    }

}
