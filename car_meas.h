/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef CAR_MEAS_H_INCLUDED
#define CAR_MEAS_H_INCLUDED

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "adc.h"
#include "wdt_timer.h"
#include "delay.h"

/* definice automobilu */

// seat arosa 1.0 MPi AER
#define AROSA_INJ     963
#define AROSA_V       4
#define AROSA_IMP     400


#define TANK_CH CH5 // definice kanalu ADC pro mereni
#define V12_CH  CH4

/*  Ux = R2/(R1+R2)Uin
*/
#define TANK_R2 18          // odpory delice napeti pro AD nadrz
#define TANK_R1 98

#define TANKM_TIME 5        /* merici cas 5* 0.25s */

#define TANKV_MAX 15        /* pocet vzorku medianu */

#define V12_R2    47            // odpory delice pro palubni sit
#define V12_R1    999
#define V12_TIME  2

#define TICKS_MAX     20000    /* maximalni namereny vstrik kdyz automobil stoji */

#define TANK_MODE_COMP 1
#define TANK_MODE_STD  0


typedef uint64_t inj_t;         /* 2^64 mereni vstrikovace */
typedef uint32_t odo_t;         /* 2^32 impuls prevodovky */
typedef uint16_t ipm_t;         /* impuls / 100metru */
typedef uint16_t ccmmin_t;        /* ccm / min */

typedef uint8_t speed_t;
typedef uint16_t fuel_t;
typedef uint32_t m_t ;
typedef uint8_t tank_t;
typedef uint16_t mv_t;

#define EE_INIT_B 'T'
typedef struct{

    uint8_t init;
    tank_t litr;
    mv_t mvolt;

}calctank_t;
#define CALCT_CNT 5

typedef struct{

    uint8_t init;       // init bajt pro init eeprom

    inj_t AvgInj;

    inj_t TripInj;

    inj_t TotInj;

    odo_t AvgImp;
    odo_t TripImp;
    odo_t TotImp;

    ipm_t CalibMeters;      // 500 = 500imp/100m
    ccmmin_t CalibInjFlow; // 3500 = 350,0 ccm/min

    tank_t TankL;
    tank_t CalibL1;     // 10 = 10L
    tank_t CalibL2;     // 10=10L
    mv_t CalibV1;       //1250 = 1250mV
    mv_t CalibV2;       // 1250 = 1250mV

    uint8_t TankMode;

    calctank_t TankCalib[CALCT_CNT];

    uint8_t NumV;

    /* tohle by tu nemelo byti protoze je to prakticky nastaveni GUI a ne mereni :( */

    uint8_t SleepTime;

    uint16_t calib_volt;

    uint8_t menu1;

    uint8_t sleepFlag;

}cm_t;


void InitCMeas(void);
void ClearTankMeas(void);       // nuluje prumerovani nadrze
uint8_t RetTankMeasMax(void);   // vraci 0/1 zda je je plny buffer vzorku paliva
void CMeasPoll(void);       // polling ktery je v v wdt timer preruseni - odber namerenych hodnot 1x za cca 1s

void StoreCMeas(void); // ulozi do eeprom

fuel_t GetCurrFuelCons(void);       // vrati aktualni spotrebu paliva v xx,x litrech/ 100km
fuel_t GetAvgFuelCons(void);        // vrati prumernou ve stejnem formatu

fuel_t GetCurrFuelConsL(void);      // vrati aktualni ve formatu xx,x l/h

fuel_t GetTripFuelCons(void);       // vrati prumernou spotrebu z trasy v xx,x l/100km
fuel_t GetTotFuelCons(void);        // vrati celkovou spotrebu xx,x l /100km

fuel_t GetCurrFuel(void);            // vrati aktualne spotrebovane xx L paliva
fuel_t GetTripFuel(void);               // vrati spotrebovane palivo v L za trasu

m_t GetCurrMeters(void);            // vrati aktualne ujete metry
m_t GetTripMeters(void);            // vrati metry ujete v trase
m_t GetTotMeters(void);             // vrati metry celkove od pocatku resetu PP

tank_t GetTankL(void);          // vrati aktualni stav nadrze v L -- prumeruje plovoucim prumerem
uint16_t GetMaxDis(void);       // vrati max dojezd vzhledem ke stavu nadrze

m_t GetActMeters(void);     // vrati aktualni metry ujete za 1s cca

uint8_t IsRunning(void);        // detekce zda motor bezi ci auto jede

uint16_t GetInjCnt(void); // vraci aktualni stav nacitanych jednotek vstrikovace
uint16_t GetImpCnt(void);
void V12MeasPoll(void);     // odmery napeti palubni site - nutno pridat do main
mv_t GetV12mV(void);        //  vraci napeti palubni site v mV
mv_t RetTmV(uint16_t ad);   // raci napeti nadrze v mV
mv_t RetTankmV(void);       // vraci prumerovane napeti nadrze v mV
void TankMeasPoll(void);        // odmery nadrze - nutno pridat do main

fuel_t RetFuelConsL(void);      // vraci spotrebu paliva aktualni v  xx,x L/h
fuel_t RetFuelCons(odo_t * imp, inj_t* inj);      // funkce ktera vypocitava spotrebu paliva v xx,x l/100km -- pro interni pouziti knihovny

m_t RetMeters(odo_t* imp);      // vraci impulzy prepocitane na metry

ipm_t GetCalibMeters(void); // vraci kalib. konstantu pro metry
ccmmin_t GetCalibInjFlow(void);     // vraci kalib. konst pro vstrik
void SetCalibMeters(ipm_t cm);  // nastavuje kalib. metry
void SetCalibInjFlow(ccmmin_t ij);  // nastavuje kalib vstrik
uint8_t GetCalibValve(void);    // vraci pocet valcu
void SetCalibValve(uint8_t v);  // nastavuje pocet valcu
//mv_t GetCalibV1(void);  // vraci kalib napeti c1 pro nadrz mV
//mv_t GetCalibV2(void);  // vraci c2 napeti pro nadrz mV
//void SetCalibV1(mv_t v);    // nastavuje c1 mV
//void SetCalibV2(mv_t v);    // nastavuje c2 mV
//tank_t GetCalibL1(void);    // vraci c1 konst. v L
//tank_t GetCalibL2(void);    // vraci c2 konst. v L
//void SetCalibL1(tank_t l);  // nastavuje c1 v L
//void SetCalibL2(tank_t l);  // nastavuje c2 v L

tank_t GetCalibL(void);     // vraci kalib konst objemu nadrze v L
void SetCalibL(tank_t l);      // nastavuje kalib kosnt objemu nadrze v L


void ClearCMeas(void);  // maze kompletne eeprom data -- pro reset
void ClearAvgMeas(void); // maze Avg hodnoty
void ClearTripMeas(void); // maze trasa trip hodnoty

tank_t GetCalibLx(uint8_t i);
mv_t GetCalibVx(uint8_t i);
uint8_t GetCalibIx(uint8_t i);
void SetCalibLx(uint8_t i,tank_t litr);
void SetCalibVx(uint8_t i,mv_t mv);
void SetCalibIx(uint8_t i,uint8_t init);

uint16_t GetCalibVolt(void);
void SetCalibVolt(uint16_t calib);

uint8_t GetTankMode(void);
void SetTankMode(uint8_t set);

void initComparator(void);
void disComparator(void);

void ADCMeasPoll(void);

uint16_t GetActSpeed(void);

void setSleepTime(uint8_t time);
uint8_t getSleepTime(void);
void setMenu1(uint8_t menu);
uint8_t getMenu1(void);

void setSleepFlag(void);
void clearSleepFlag(void);
uint8_t isSleeFlagEnabled(void);

#endif // CAR_MEAS_H_INCLUDED

