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

/* definice automobilu */ // presunuto do inicializace pro podporu více aut
#define XANTIA

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
typedef uint32_t time_t;         /* cas jizdy pro vypocet prumernych rychlosti */

typedef float speed_t;
typedef float fuel_t;
typedef uint32_t dist_m_t;
typedef float dist_km_t;
typedef float tank_t;
typedef uint16_t mv_t;
typedef float v_t;
typedef float t_t;
typedef float h_t;

#define EE_INIT_B '#'

typedef struct {
	
	uint8_t init;       // init bajt pro init eeprom

	uint8_t SleepTime;
	uint16_t VoltmeterCalibration;
	uint8_t DefaultSubDispPref;
	uint8_t Sensors;
	uint8_t SubDisplayChangeSpeed;
	uint8_t WideMode;

} bcdata_t;
 
typedef struct{

    uint8_t init;
    tank_t litr;
    mv_t mvolt;

}calctank_t;
#define CALCT_CNT 5

typedef struct{

    uint8_t init;       // init bajt pro init eeprom

    uint8_t NumC; // number of cylinders
    ccmmin_t CalibInjFlow; // 3500 = 350,0 ccm/min
    ipm_t CalibMeters;      // 500 = 500imp/100m
    uint16_t TankL;
    uint8_t TankMode;
    calctank_t TankCalib[CALCT_CNT];
	uint16_t CalibSpeedRatio;
	uint8_t TankFuelSrc;

}cardata_t;

//typedef enum {
	//Current, Trip, Tank, Total;
	//} TripType;
//
//typedef struct{
//
	//inj_t Inj;
	//odo_t Imp;
	//time_t Time;
//
//}calctank_t;

typedef struct{

	uint8_t init;       // init bajt pro init eeprom

    inj_t CurrInj;
    inj_t TripInj;
    inj_t TankInj;
    inj_t TotInj;

    odo_t CurrImp;
    odo_t TripImp;
    odo_t TankImp;
    odo_t TotImp;
	
	time_t CurrTime;
	time_t TripTime;
	time_t TankTime;
	time_t TotTime;

}statsdata_t;

void InitDataAndApp(void);
void ClearTankMeas(void);       // nuluje prumerovani nadrze
uint8_t RetTankMeasMax(void);   // vraci 0/1 zda je je plny buffer vzorku paliva
void StatsPoll(void);       // polling ktery je v wdt timer preruseni - odber namerenych hodnot 1x za cca 1s

void StoreBCData(void); 
void StoreCarData(void); 
void StoreStatsData(void); 
void StoreData(void); 

fuel_t GetInstFuelCons(void);   // vrati aktualni spotrebu paliva v xx,x litrech/ 100km
fuel_t GetInstFuelConsL(void);  // vrati aktualni ve formatu xx,x l/h
fuel_t GetCurrFuelCons(void);   // vrati prumernou ve stejnem formatu
fuel_t GetTripFuelCons(void);   // vrati prumernou spotrebu z trasy v xx,x l/100km
fuel_t GetTankFuelCons(void);   // vrati prumernou spotrebu z trasy nadrze v xx,x l/100km
fuel_t GetTotFuelCons(void);    // vrati celkovou spotrebu xx,x l /100km

fuel_t GetCurrFuel(void);      // vrati aktualne spotrebovane xx L paliva
fuel_t GetTripFuel(void);      // vrati spotrebovane palivo v L za trasu
fuel_t GetTankFuel(void);      // vrati spotrebovane palivo v L za trasu nadrze
fuel_t GetTotFuel(void);      // vrati celkove spotrebovane palivo

dist_km_t GetCurrKMeters(void);       // vrati aktualne ujete metry
dist_km_t GetTripKMeters(void);       // vrati metry ujete v trase
dist_km_t GetTankKMeters(void);       // vrati metry ujete v trase nadrze
dist_km_t GetTotKMeters(void);        // vrati metry celkove od pocatku resetu PP
odo_t * GetTripImp(void);
odo_t * GetTankImp(void);
odo_t * GetTotImp(void);

time_t GetCurrTime(void);       // vrati aktualne ujete sekundy
time_t GetTripTime(void);       // vrati sekundy ujete v trase
time_t GetTankTime(void);       // vrati sekundy ujete v trase nadrze
time_t GetTotTime(void);        // vrati sekundy celkove od pocatku resetu PP

tank_t GetTankLMeas(void);      // vrati aktualni stav nadrze v L -- prumeruje plovoucim prumerem
tank_t GetTankLCalc(void);      // vrati aktualni stav nadrze v L vypoctem ze spotrebovaneho paliva na trase "Nadrz"
tank_t GetTankL(void);          // vrati aktualni stav nadrze v L podle aktuálního nastavení (buï mìøením, nebo výpoètem)
dist_km_t GetRange(fuel_t cons);// vrati max dojezd vzhledem ke stavu nadrze a udane spotrebe
dist_km_t GetInstRange(void);   // vrati max dojezd vzhledem ke stavu nadrze a okamzite spotrebe
dist_km_t GetCurrRange(void);   // vrati max dojezd vzhledem ke stavu nadrze a prùmerne spotrebe za jizdu
//TODO
//uint16_t GetTotRange(void);    // vrati max dojezd vzhledem ke stavu nadrze a celkove prumerne spotrebe

dist_km_t GetInstKMeters(void);        // vrati aktualni kilometry ujete za 1s cca
h_t GetInstRemHrs(void);		 // vrátí zbývající dobu bìhu motoru

uint8_t IsRunning(void);        // detekce zda motor bezi ci auto jede
uint8_t IsWide(void);           // zda pouzívat layout pro siroky displej (LS)
void RefreshWideFlag(void);

inj_t GetInjCnt(void); // vraci aktualni stav nacitanych jednotek vstrikovace
odo_t GetImpCnt(void); // vraci aktualni stav nacitanych jednotek impulzù ujeté vzdálenosti
void V12MeasPoll(void);     // odmery napeti palubni site - nutno pridat do main
mv_t GetV12mV(void);        //  vraci napeti palubni site v mV
v_t GetV12V(void);          //  vraci napeti palubni site ve V s pøesností na DECIMALS
mv_t RetTmV(uint16_t ad);   // vraci napeti nadrze v mV
mv_t RetTankmV(void);       // vraci prumerovane napeti nadrze v mV
void TankMeasPoll(void);        // odmery nadrze - nutno pridat do main

fuel_t RetInstFuelConsL(void);      // vraci spotrebu paliva aktualni v  xx,x L/h
fuel_t RetFuelCons(odo_t * imp, inj_t* inj);      // funkce ktera vypocitava spotrebu paliva v xx,x l/100km -- pro interni pouziti knihovny

dist_m_t RetMeters(odo_t* imp);      // vraci impulzy prepocitane na metry
dist_km_t RetKMeters(odo_t* imp);      // vraci impulzy prepocitane na kilometry

ipm_t GetCalibMeters(void); // vraci kalib. konstantu pro metry
ccmmin_t GetCalibInjFlow(void);     // vraci kalib. konst pro vstrik
void SetCalibMeters(ipm_t cm);  // nastavuje kalib. metry
void SetCalibInjFlow(ccmmin_t ij);  // nastavuje kalib vstrik
uint8_t GetCalibCyls(void);    // vraci pocet valcu
void SetCalibCyls(uint8_t v);  // nastavuje pocet valcu
//mv_t GetCalibV1(void);  // vraci kalib napeti c1 pro nadrz mV
//mv_t GetCalibV2(void);  // vraci c2 napeti pro nadrz mV
//void SetCalibV1(mv_t v);    // nastavuje c1 mV
//void SetCalibV2(mv_t v);    // nastavuje c2 mV
//tank_t GetCalibL1(void);    // vraci c1 konst. v L
//tank_t GetCalibL2(void);    // vraci c2 konst. v L
//void SetCalibL1(tank_t l);  // nastavuje c1 v L
//void SetCalibL2(tank_t l);  // nastavuje c2 v L
uint16_t GetCalibSpeedRatio(void);   // získání pomìru zobrazované rychlosti vùèi namìøeným metrùm
void SetCalibSpeedRatio(uint16_t ratio); // nastavení pomìru zobrazované rychlosti vùèi namìøeným metrùm

tank_t GetCalibTankL(void);     // vraci kalib konst objemu nadrze v L
void SetCalibTankL(tank_t l);      // nastavuje kalib kosnt objemu nadrze v L

void ClearBCData(void);
void ClearCarData(void);
void ClearStatsData(void);
void ClearData(void);  // maze kompletne eeprom data -- pro reset

void ClearCurrData(void); // maze hodnoty aktuální jízdy
void ClearTripData(void); // maze hodnoty trasy
void ClearTankData(void); // maze hodnoty trasy nadrze

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

uint8_t GetTankFuelSrc(void);
void SetTankFuelSrc(uint8_t set);

uint8_t GetWideMode(void);
void SetWideMode(uint8_t set);

void initComparator(void);
void disComparator(void);

void ADCMeasPoll(void);

speed_t GetInstSpeed(void);
speed_t GetSpeedFor(dist_km_t kms, time_t secs);
speed_t GetCurrSpeed(void);
speed_t GetTripSpeed(void);
speed_t GetTankSpeed(void);
speed_t GetTotSpeed(void);

void setSleepTime(uint8_t time);
uint8_t getSleepTime(void);
void SetDefSubDisp(uint8_t menu);
uint8_t GetDefSubDisp(void);
uint8_t GetSubDispChangeSpeed(void);
void SetSubDispChangeSpeed(uint8_t speed);

// temperature stuff
#include "ds18b20.h"

#define SENSOR_NO       0
#define SENSOR_OUT      1
#define SENSOR_IN       2
#define SENSOR_OUT_IN   3

/* 10*120ms temp Data in main()*/
void SensorsPoll(void);

/* user funcs return temps */
float RetSensorIn(void);
float RetSensorOut(void);
void SensorsSet(uint8_t set);
uint8_t SensorsGet(void);
void SensorInit(void);

#endif // CAR_MEAS_H_INCLUDED

