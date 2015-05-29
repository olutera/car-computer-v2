/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/

#include "car_meas.h"
#include <string.h>
#include <stdio.h>
#include "hd44780.h"
#include "global.h"
#include <math.h>


//#define USART_DEBUG - moved to global.h
#ifdef USART_DEBUG
#include "uart.h"
#include <stdio.h>
#include <avr/pgmspace.h>
#endif

volatile uint32_t Counter1Ticks=0;
volatile uint16_t Counter0Ticks=0;

volatile uint8_t int0_flag=0;

// section positions are specified in the linker settings
bcdata_t BCData;
bcdata_t eBCVals __attribute__ ((section(".BCData"))); // 8 bytes so far, reserver 128bytes
cardata_t CarData;
cardata_t eCarVals __attribute__ ((section (".CarData"))); // 29 bytes so far, reserved 128bytes
statsdata_t StatsData;
statsdata_t eStatsVals __attribute__ ((section (".StatsData"))); // 68 bytes so far, reserved 128bytes

volatile uint8_t Tim1_OVF=0;

ISR(TIMER1_OVF_vect)
{
    Tim1_OVF++;
}

void Init50kPD3(void)
{
    DDRD |= 1 << PD3;
    PORTD &= ~(1<<PD3);

    TCCR2B=0;

    TCNT2=0;
    TCNT2=0;

    OCR2A = 79; /* 100kHz */

    TCCR2A = 1<< COM2B0 | 1<<WGM21;

    TCCR2B = 1 << CS20;

    /* obcas ma timer tendenci nabehnout na spatne frekvenci - zejmena pokud doslo k suspendu a naslednemu odpojeni/pripojeni napajeni - snad vyreseno */
}

void InitDataAndApp(void)
{

    if(eeprom_read_byte(&eBCVals.init)!=EE_INIT_B)
    {
        BCData.init = EE_INIT_B;

        BCData.VoltmeterCalibration = 999;
        BCData.DefaultSubDispPref = 0;
        BCData.SleepTime = 30;
        BCData.Sensors = 0;
        BCData.SubDisplayChangeSpeed = 16;

        #ifdef XANTIA
        BCData.VoltmeterCalibration = 1007;
        BCData.DefaultSubDispPref = 0;
        BCData.SleepTime = 30;
        BCData.Sensors = 1;
        BCData.SubDisplayChangeSpeed = 8;
        #endif

        StoreBCData();
    }

    if(eeprom_read_byte(&eCarVals.init)!=EE_INIT_B)
    {
        // init with default/empty values
        CarData.init = EE_INIT_B;

        CarData.NumC = 4;
        CarData.CalibInjFlow = 100;
        CarData.CalibMeters = 400;
        CarData.CalibSpeedRatio = 100;

        CarData.TankL   = 0;

        for(uint8_t i=0; i<CALCT_CNT; i++)
        {
            CarData.TankCalib[i].init = 0;
            CarData.TankCalib[i].mvolt = 0;
            CarData.TankCalib[i].litr = 0;
        }

        CarData.TankMode = TANK_MODE_STD;

        #ifdef XANTIA
        CarData.NumC = 4;
        CarData.CalibInjFlow = 1820;
        CarData.CalibMeters = 500;
        CarData.CalibSpeedRatio = 91;
        CarData.TankL = 65;
        CarData.TankCalib[0].init = EE_INIT_B;
        CarData.TankCalib[0].mvolt = 3200;
        CarData.TankCalib[0].litr = 0;
        CarData.TankCalib[CALCT_CNT-1].init = EE_INIT_B;
        CarData.TankCalib[CALCT_CNT-1].mvolt = 620;
        CarData.TankCalib[CALCT_CNT-1].litr = 65;
        #endif

        StoreCarData();
    }

    if(eeprom_read_byte(&eStatsVals.init)!=EE_INIT_B)
    {
        StatsData.init = EE_INIT_B;

        StatsData.CurrInj = 0;
        StatsData.TripInj = 0;
        StatsData.TankInj = 0;
        StatsData.TotInj  = 0;

        StatsData.CurrImp = 0;
        StatsData.TripImp = 0;
        StatsData.TankImp = 0;
        StatsData.TotImp  = 0;

    StoreStatsData();
    }

    eeprom_read_block(&BCData,&eBCVals,sizeof(bcdata_t));
    eeprom_read_block(&CarData,&eCarVals,sizeof(cardata_t));
    eeprom_read_block(&StatsData,&eStatsVals,sizeof(statsdata_t));

    Init50kPD3();

    TCNT1=0;
    TIMSK1 |= 1 << TOIE1;        /* TIMER1_OVF_vect */

    TCCR1B = 1<< CS11 | 1 << CS12;      /* counter 1 16bit PD5*/

    DDRD &= ~(1<<PD4);      /* input T0 */
    PORTD |= 1 << PD4;      /* pullup T0 */

    TCNT0=0;
    TIMSK0 |= 1 << TOIE0;        /* TIMER0_OVF_vect */

    TCCR0B=1<<CS02|1<<CS01|1<<CS00;    /* rissing edge T0*/

    if(CarData.TankMode == TANK_MODE_COMP) initComparator();
    else disComparator();

    /* osetrime narychlo chybnou hodnotu po prehravani firmware a zmene eeprom */
    if(getSleepTime() > 60 || getSleepTime() ==0) setSleepTime(30);
    if(GetDefSubDisp() > 6) SetDefSubDisp(4);

    RefreshWideFlag(); // reads flash data into volatile var

    sei(); // enable global interrupts
}

volatile uint16_t TankV[TANKV_MAX];
volatile uint8_t TankSampleMax=0;

uint8_t TankVindex =0;
uint8_t TankMeasTime=0;
uint8_t V12MeasTime=0;
uint16_t V12=0;
uint8_t compStoppedNow = 0;

ISR(ANALOG_COMP_vect)
{

    uint8_t i;
    uint16_t meas;

    disComparator();

    StartAD(TANK_CH);           /* prvni mereni */
    ReadAD(TANK_CH);

    TankV[TankVindex]=0;

    for(i=0; i<10; i++)
    {

        StartAD(TANK_CH);

        meas=ReadAD(TANK_CH);

        if(meas > TankV[TankVindex]) TankV[TankVindex]=meas;    // misto prumeru ted hledam maximum

    };

    TankVindex++;

    if(TankVindex==TANKV_MAX)
    {
        TankVindex=0;
        TankSampleMax=1;
    }

    compStoppedNow = 1;

}

void initComparator(void)
{
    ACSR = 0;
    ADCSRA = 0;     // chcipneme DAC
    ADMUX = TANK_CH;        // vyber vstupu nadrze
    ADCSRB |=1<< ACME;  //multiplex
    ACSR = 1 << ACIS1 ; // enable interrupt , falling edge
    DIDR1 |= 1<< AIN0D;
    ACSR |= 1 << ACIE;
}

void disComparator(void)
{
    ACSR = 0;
    ADCSRB&=~(1<<ACME);  // multiplex
    ACSR|=1<<ACD;       // disable comparator
    DIDR1 |= 1<< AIN0D;

}


inline void ADCMeasPoll(void)
{

    uint8_t compState = 0;

    if(CompWdTimer(V12MeasTime,TANKM_TIME))
    {

        if(bit_is_clear(ACSR,ACD) && CarData.TankMode == TANK_MODE_COMP)
        {

            disComparator();

            compState = 1;

        }

        V12MeasTime = RetWdTimer();

        StartAD(V12_CH);
        ReadAD(V12_CH);

        StartAD(V12_CH);
        V12 = ReadAD(V12_CH);

        if(compState)
        {

            initComparator();

        }

    }

    if(CarData.TankMode== TANK_MODE_STD)
    {

        if(CompWdTimer(TankMeasTime,TANKM_TIME))
        {

            TankMeasTime = RetWdTimer()    ;

            StartAD(TANK_CH);           /* prvni mereni */
            ReadAD(TANK_CH);

            StartAD(TANK_CH);

            TankV[TankVindex++]=ReadAD(TANK_CH);


            if(TankVindex==TANKV_MAX)
            {
                TankVindex=0;
                TankSampleMax=1;

            }

        }


    }
    else
    {

        if(compStoppedNow)
        {

            TankMeasTime = RetWdTimer();
            compStoppedNow = 0;

        }

        if(CompWdTimer(TankMeasTime,TANKM_TIME) && bit_is_set(ACSR,ACD) )
        {

            initComparator() ;

        }


    }
}

inline void ClearTankMeas(void)
{
    TankVindex=0;
    TankSampleMax=0;
    if(CarData.TankMode == TANK_MODE_COMP) initComparator();
}

inline uint8_t RetTankMeasMax(void)
{
    return TankSampleMax;
}

volatile uint8_t Tim0_OVF=0;

ISR(TIMER0_OVF_vect)
{
    Tim0_OVF++;
}

volatile uint8_t is_running = 0;
volatile uint8_t is_wide;

/* volana z interruptu casovacce */
void StatsPoll(void)
{
    Counter1Ticks = (uint32_t)Tim1_OVF*(uint32_t)65536ul + (uint32_t)TCNT1;
    TCNT1 = 0;
    Tim1_OVF=0;
    Counter1Ticks/=2;

    Counter0Ticks = (uint16_t)Tim0_OVF*(uint16_t)256u + (uint16_t)TCNT0;
    TCNT0=0;
    Tim0_OVF=0;

#ifdef USART_DEBUG
char debug[300];
uart_init(UART_BAUD_SELECT(115200,F_CPU));
sprintf_P(debug,PSTR("DEBUG:StatsPoll():Counter1Ticks=%ul;Counter0Ticks=%ul;\n"),Counter1Ticks,Counter0Ticks);
uart_puts(debug);
#endif // USART_DEBUG

    /* osetreni maxima - pri vypnutem motoru, nektere vstrikovace spinaji vstupni logiku do log 0 a nechtelo to bez tohoto uspavat */
    if( (Counter0Ticks!=0) || (/*Counter1Ticks<TICKS_MAX && */Counter1Ticks!=0)  )
    {
        // omit one-impulse data which may occur during ignition start-up
        if (Counter1Ticks == 1) {
          Counter1Ticks = 0;
        }

        if (Counter0Ticks == 1) {
            Counter0Ticks = 0;
        }

        StatsData.CurrInj += Counter1Ticks;
        StatsData.TripInj += Counter1Ticks;
        StatsData.TankInj += Counter1Ticks;
        StatsData.TotInj  += Counter1Ticks;

        StatsData.CurrImp += Counter0Ticks;
        StatsData.TripImp += Counter0Ticks;
        StatsData.TankImp += Counter0Ticks;
        StatsData.TotImp  += Counter0Ticks;

        if (Counter1Ticks != 0 || Counter0Ticks != 0) {
            // we are really measuring running engine or moving car
            //TODO - teï se to spoléhá jen na watchdog timer a na volání StatsPoll jednou za cca 1s
            StatsData.CurrTime += 1;
            StatsData.TripTime += 1;
            StatsData.TankTime += 1;
            StatsData.TotTime  += 1;

            is_running=1;
        }
    }
    else
    {
        is_running=0;
        Counter1Ticks=0;
        Counter0Ticks=0;
    }
}

mv_t RetTmV(uint16_t ad)
{

    uint32_t sum=ad;
    sum = ((uint32_t)REF_VOLT*(uint32_t)sum*10)/1024;

    return ((sum*(uint32_t)(TANK_R1+TANK_R2))/TANK_R2/10);

}

mv_t RetTankmV(void)
{
    uint8_t disACIE = 0;

    uint16_t val=TankV[0];
    uint8_t index,i,j;
    uint16_t sort[TANKV_MAX];

    if(TankSampleMax==1)index = TANKV_MAX;
    else index = TankVindex+1;

    if(CarData.TankMode == TANK_MODE_COMP && ACSR&(1<<ACIE) )
    {
        ACSR &=~(1<<ACIE); // zakaz int comparatoru
        disACIE = 1;

    }

    memcpy(sort,(uint16_t*)TankV,2*index);

    for(i=0; i<index; i++)
    {

        for(j=i+1; j<index; j++)
        {

            if(sort[i]>sort[j])
            {

                val=sort[i];
                sort[i]=sort[j];
                sort[j]=val;

            }

        }

    }

    if(disACIE)ACSR|=1<<ACIE;

    return RetTmV(sort[index/2]);
}

mv_t GetV12mV(void)
{

    uint32_t mv;

    mv = ((uint32_t)V12 * REF_VOLT)/1024;

    /*return ((mv*(uint32_t)(V12_R1+V12_R2))/V12_R2); */
    return ((mv*(uint32_t)(BCData.VoltmeterCalibration+V12_R2))/V12_R2);

}

v_t GetV12V(void)
{
    return GetV12mV()/1000.0f;
}

/* pridano 8.10.2011 - implementace po castech linearni aproximace nadrze */
tank_t GetTankLMeas(void)
{

    int32_t l2=0;
    int32_t l1=0;
    int32_t mv2=0;
    int32_t mv1=0;
    int32_t mv = RetTankmV();
    uint8_t i=0;

    while(i<CALCT_CNT)
    {

        if(CarData.TankCalib[i].init == EE_INIT_B)
        {

            if(CarData.TankCalib[i].mvolt>mv)
            {

                break;
            };

        }

        i++;

    }
    /* pokud je namerena hodnota proste vetsi nez vsechny ulozene konstanty, pak pouzij nejvyssi ulozene konstanty tj posledni v poli*/
    if(i==CALCT_CNT)
    {
        i--;
        while(i>0)
        {
            /* najde nejvyssi horni*/
            if(CarData.TankCalib[i].init == EE_INIT_B) break;
            i--;
        }

        l2 = CarData.TankCalib[i].litr;
        mv2 = CarData.TankCalib[i].mvolt;
        i--;
        while(i>0)
        {
            /* najde druhou nejvyssi horni*/
            if(CarData.TankCalib[i].init == EE_INIT_B) break;
            i--;
        }
        /* pokud je ovsem i==0 a zaroven neni inicializovane pole pak je problem a nelze pocitat protoze nejsou ulozene konstanty*/
        if(i==0&&CarData.TankCalib[i].init!=EE_INIT_B) return 0;

    }
    else
    {
        uint8_t bck = i;
        l2 = CarData.TankCalib[i].litr;
        mv2 = CarData.TankCalib[i].mvolt;
        if(i>0)
        {
            i--;
            while(i>0)
            {

                if(CarData.TankCalib[i].init == EE_INIT_B) break;
                i--;

            }

            if(i==0&&CarData.TankCalib[i].init!=EE_INIT_B)      /* tohle je debilne naprogramovane takze workaround kdyz neexistuje inicializovana nizsi hodnota, je treba pocitat z vyssich hodnot nez vracet 0*/
            {

                i=bck;
                goto workaround;

                //return 0;
            }

            l1=CarData.TankCalib[i].litr;
            mv1=CarData.TankCalib[i].mvolt;

        }
        else
        {
workaround: //uprava pokud nulta neexistuje
            /* tzn ze nulta hodnota je jiz vetsi nez ta namerena takze potrebujem dalsi vyssi hodnotu */
            l1=l2;
            mv1=mv2;
            i++;
            while(i<CALCT_CNT)
            {
                if(CarData.TankCalib[i].init == EE_INIT_B) break;
                i++;
            }

            if(i>=CALCT_CNT) return 0;

            l2 = CarData.TankCalib[i].litr;
            mv2 = CarData.TankCalib[i].mvolt;
        }
    }
    int32_t k = ((l2 - l1)*10000);

    k /=(mv2-mv1);

    int32_t q = (l1*10000)-(mv1*k);

    k = ( (k*mv)+q )/1000;

    //zaokrouhleni
    if(k<0) k = 0;
    //else if(k%10 > 4) k+=10;
    tank_t f = k/10.0f;

return  (tank_t)(f > CarData.TankL ? CarData.TankL : f);
}

tank_t GetTankLCalc(void)
{
    return GetCalibTankL()-GetTankFuel();
}

tank_t GetTankL(void)
{
    if (GetTankFuelSrc()) // calculation
    {
        return GetTankLCalc();
    }
    else
    {
        return GetTankLMeas();
    }
}

dist_km_t GetRange(fuel_t cons)
{
    if(cons<=0) return 0;
    return (GetTankL()/cons)*100;
}

dist_km_t GetInstRange(void)
{
    return GetRange(GetInstFuelCons());
}

dist_km_t GetCurrRange(void)
{
    return GetRange(GetCurrFuelCons());
}

h_t GetInstRemHrs(void) {
    fuel_t cons = GetInstFuelConsL();
    if (cons <= 0) {
        return 0;
    }
    return (h_t)(GetTankL()/cons);
}

fuel_t GetFuel(inj_t *inj)
{
    if(*inj ==0 || CarData.CalibInjFlow ==0 || CarData.NumC ==0) return 0.0f;

    /* CalibInjFlow format 3500 = 350,0 ccm/min
        NumC 4 = 4 valce
        inj 50 000 = 1s

    */
    ////        32         64      64             64             64                        64                                64         30
    //return (fuel_t)( (inj_t)(((inj_t)*inj/(inj_t)60)*(uint64_t)CarData.NumC*(uint64_t)CarData.CalibInjFlow)/((inj_t)(500000000UL/PRECISION)) ); // 50000000UL -  modified to enhance precision to two decimal places
            //64
    //return (fuel_t)((*inj/5OOOO.0f)*(CarData.CalibInjFlow/600.0f)*0.001f*CarData.NumC);
    return (fuel_t)((*inj*CarData.CalibInjFlow*CarData.NumC)/30000000000.0f);
}

inline fuel_t GetCurrFuel(void)
{
    return GetFuel(&StatsData.CurrInj);
}

inline fuel_t GetTripFuel(void)
{
    return GetFuel(&StatsData.TripInj);
}

inline fuel_t GetTankFuel(void)
{
    return GetFuel(&StatsData.TankInj);
}

inline fuel_t GetTotFuel(void)
{
    return GetFuel(&StatsData.TotInj);
}

inline void StoreBCData(void)
{
    eeprom_update_block(&BCData,&eBCVals,sizeof(bcdata_t));
}

inline void StoreCarData(void)
{
    eeprom_update_block(&CarData,&eCarVals,sizeof(cardata_t));
}

inline void StoreStatsData(void)
{
    eeprom_update_block(&StatsData,&eStatsVals,sizeof(statsdata_t));
}

inline void StoreData(void)
{
    StoreBCData();
    StoreCarData();
    StoreStatsData();
}

dist_m_t RetMeters(odo_t * imp)
{

    if (CarData.CalibMeters == 0 || *imp == 0) return 0;

    dist_m_t m=0;

    /*if( (*imp)>((odo_t)CarData.CalibMeters*(odo_t)1000))    return (    ((*imp) / (odo_t)CarData.CalibMeters)* (odo_t)100);

    else return (    ((*imp) * (odo_t)100)  / (odo_t)CarData.CalibMeters);*/
    if(*imp < 42000000UL)
    {
        m=(((*imp) * (odo_t)100) / (odo_t)CarData.CalibMeters);
    }
    else
    {
        m=(((*imp) / (odo_t)CarData.CalibMeters) * (odo_t)100);
    }
    return m;
}

dist_km_t RetKMeters(odo_t * imp) {
    return RetMeters(imp)/1000.0f;
}

fuel_t RetFuelCons(odo_t *imp, inj_t *inj)
{

    if( *imp<4 || *inj ==0 || CarData.CalibInjFlow == 0 || CarData.NumC == 0) return 0;

    /* CalibInjFlow format 3500 = 350,0 ccm/min; NumC 4 = 4 valce; inj 50 000 = 1s */

    /* uprava nejspise neco na zpusob ukladani minut v 32bit promenne + druhy 32bit prom na zbytky*/
    //return ((*inj * (uint64_t)CarData.NumC * (uint64_t)CarData.CalibInjFlow)/(uint64_t)RetMeters(imp))/((uint64_t)300000/PRECISION); // 30000 - úprava na pøesnost na dvì desetinná místa
    return (fuel_t)((GetFuel(inj)*100) / RetKMeters(imp));
}

fuel_t RetInstFuelConsL(void)
{
    if( Counter1Ticks == 0 || CarData.NumC == 0 || CarData.CalibInjFlow == 0) return 0;

    /* Counter / 50 000 -> (50khz) mam sekundy. *valce* InjFlow v (ccm/min)/10 / 60 / 1000 ( na dm2=litry) * 10 ( presnost na jedno misto) *3600 -> xx,x L za hodinu */

    //return ( (((((uint64_t)Counter1Ticks * 3 * (uint64_t)CarData.NumC)/250) * (uint64_t)CarData.CalibInjFlow/10))/(10000/PRECISION));
    inj_t ticks = GetInjCnt();
    return GetFuel(&ticks) * 3600.0f;
}

inline fuel_t GetInstFuelCons(void)
{
    inj_t ticks = GetInjCnt();
    odo_t imp = GetImpCnt();

    return RetFuelCons(&imp,&ticks);
}

// 2014-06-23 zmìna z uint16_t, možná zde docházelo k pøeteèení pøi vysoké spotøebì
inj_t GetInjCnt(void)
{
    return Counter1Ticks;
}

odo_t GetImpCnt(void)
{
    return Counter0Ticks;
}

inline fuel_t GetInstFuelConsL(void)
{
    return RetInstFuelConsL();
}

inline fuel_t GetCurrFuelCons(void)
{
    return RetFuelCons(&StatsData.CurrImp,&StatsData.CurrInj);
}

inline fuel_t GetTripFuelCons(void)
{
    return RetFuelCons(&StatsData.TripImp,&StatsData.TripInj);
}

inline fuel_t GetTankFuelCons(void)
{
    return RetFuelCons(&StatsData.TankImp,&StatsData.TankInj);
}

inline fuel_t GetTotFuelCons(void)
{
    return RetFuelCons(&StatsData.TotImp,&StatsData.TotInj);
}

inline dist_km_t GetCurrKMeters(void)
{
    return RetKMeters(&StatsData.CurrImp);
}

inline dist_km_t GetTripKMeters(void)
{
    return RetKMeters(&StatsData.TripImp);
}

inline dist_km_t GetTankKMeters(void)
{
    return RetKMeters(&StatsData.TankImp);
}

inline dist_km_t GetTotKMeters(void)
{
    return RetKMeters(&StatsData.TotImp);
}

inline odo_t * GetTripImp(void) {
    return &StatsData.TripImp;
}

inline odo_t * GetTankImp(void) {
    return &StatsData.TankImp;
}

inline odo_t * GetTotImp(void) {
    return &StatsData.TotImp;
}

inline dist_km_t GetInstKMeters(void)
{
    odo_t imp = GetImpCnt();
    return RetKMeters(&imp);
}

inline time_t GetCurrTime(void) {
    return (time_t)StatsData.CurrTime;
    }
    
inline time_t GetTripTime(void) {
    return (time_t)StatsData.TripTime;
}

inline time_t GetTankTime(void) {
    return (time_t)StatsData.TankTime;
}

inline time_t GetTotTime(void) {
    return (time_t)StatsData.TotTime;
}

inline uint8_t IsRunning(void)
{
    return is_running;
}

inline uint8_t IsWide(void)
{
    return is_wide;
}

inline void RefreshWideFlag(void) {
    is_wide = BCData.WideMode;
}

inline ipm_t GetCalibMeters(void)
{
    return CarData.CalibMeters;
}

inline ccmmin_t GetCalibInjFlow(void)
{
    return CarData.CalibInjFlow;
}

inline void SetCalibMeters(ipm_t cm)
{
    CarData.CalibMeters = cm;
}

inline uint16_t GetCalibSpeedRatio(void)
{
    return CarData.CalibSpeedRatio;
}

inline void SetCalibSpeedRatio(uint16_t ratio)
{
    CarData.CalibSpeedRatio = ratio;
}

inline void SetCalibInjFlow(ccmmin_t ij)
{
    CarData.CalibInjFlow = ij;
}

inline uint8_t GetCalibCyls(void)
{
    return CarData.NumC;
}

inline void SetCalibCyls(uint8_t c)
{
    CarData.NumC = c;
}

inline tank_t GetCalibLx(uint8_t i)
{
    if(i>=CALCT_CNT) return 0;
    return CarData.TankCalib[i].litr;
}

inline mv_t GetCalibVx(uint8_t i)
{
    if(i>=CALCT_CNT) return 0;
    return CarData.TankCalib[i].mvolt;
}

inline uint8_t GetCalibIx(uint8_t i)
{
    if(i>=CALCT_CNT) return 0;
    return CarData.TankCalib[i].init;
}

inline void SetCalibLx(uint8_t i,tank_t litr)
{
    if(i>=CALCT_CNT) return;
    CarData.TankCalib[i].litr=litr;
}

inline void SetCalibVx(uint8_t i,mv_t mv)
{
    if(i>=CALCT_CNT) return ;
    CarData.TankCalib[i].mvolt=mv;
}

inline void SetCalibIx(uint8_t i,uint8_t init)
{
    if(i>=CALCT_CNT) return ;
    CarData.TankCalib[i].init = init;
}

inline tank_t GetCalibTankL(void)
{
    return CarData.TankL*1.0f;
}

inline void SetCalibTankL(tank_t l)
{
    CarData.TankL = l;
}

inline void ClearBCData(void)
{
    eeprom_update_byte(&eBCVals.init,0);
}

inline void ClearCarData(void)
{
    eeprom_update_byte(&eCarVals.init,0);
}

inline void ClearStatsData(void)
{
    eeprom_update_byte(&eStatsVals.init,0);
}

inline void ClearData(void)
{
    ClearBCData();
    ClearCarData();
    ClearStatsData();
}

inline void ClearCurrData(void)
{
    cli();
    StatsData.CurrInj =  0;
    StatsData.CurrImp =  0;
    StatsData.CurrTime = 0;
    sei();

}
inline void ClearTripData(void)
{
    cli();
    StatsData.TripInj  = 0;
    StatsData.TripImp  = 0;
    StatsData.TripTime = 0;
    sei();
}

inline void ClearTankData(void)
{
    cli();
    StatsData.TankInj  = 0;
    StatsData.TankImp  = 0;
    StatsData.TankTime = 0;
    sei();
}

inline uint8_t GetTankMode(void)
{
    return CarData.TankMode;
}

inline void SetTankMode(uint8_t set)
{

    CarData.TankMode = set;
    if(set == TANK_MODE_COMP) initComparator();
    else disComparator();

}

inline uint8_t GetTankFuelSrc(void)
{
    return CarData.TankFuelSrc;
}

inline void SetTankFuelSrc(uint8_t set)
{
    CarData.TankFuelSrc = set;
}

inline uint8_t GetWideMode(void)
{
    return BCData.WideMode;
}

inline void SetWideMode(uint8_t set)
{
    BCData.WideMode = set;
}

speed_t GetInstSpeed(void)
{
    // pro optimalizaci výpoètu je pokráceno násobení impulzù x100 a dìlení CalibSpeedRation /100
    return (GetImpCnt() / (float)CarData.CalibMeters) * 3.6f /* m/s -> km/h */ * (float)CarData.CalibSpeedRatio;
}

speed_t GetSpeedFor(dist_km_t kms, time_t secs) {
    speed_t ret = 0.0f;
    if (kms > 0 && secs > 0) {
        ret = (kms*3600.0f/secs*1.0f)*(CarData.CalibSpeedRatio/100.0f); 
    }
    return ret;
}

speed_t GetCurrSpeed(void) 
{
    return GetSpeedFor(GetCurrKMeters(),GetCurrTime());
}

speed_t GetTripSpeed(void)
{
    return GetSpeedFor(GetTripKMeters(),GetTripTime());
}

speed_t GetTankSpeed(void)
{
    return GetSpeedFor(GetTankKMeters(),GetTankTime());
}

speed_t GetTotSpeed(void)
{
    return GetSpeedFor(GetTotKMeters(),GetTotTime());
}

inline uint16_t GetCalibVolt(void)
{
    return BCData.VoltmeterCalibration;
}

inline void SetCalibVolt(uint16_t calib)
{
    BCData.VoltmeterCalibration = calib;
}

inline uint8_t getSleepTime(void)
{
    return BCData.SleepTime;
}

inline void setSleepTime(uint8_t time)
{
    BCData.SleepTime = time;
}

/* zobrazovaci workaround..kazdy chce neco jineho */

inline void SetDefSubDisp(uint8_t subdispID) {
    BCData.DefaultSubDispPref = subdispID;
}

inline uint8_t GetDefSubDisp(void) {
    return BCData.DefaultSubDispPref;
}

inline uint8_t GetSubDispChangeSpeed(void) {
    return BCData.SubDisplayChangeSpeed;
}

inline void SetSubDispChangeSpeed(uint8_t speed) {
    BCData.SubDisplayChangeSpeed = speed;
}

// sensors stuff
int16_t temp_in=0;
int16_t temp_out=0;

uint8_t ConvProgrs = 0;
uint8_t TimState = 0;

inline float RetSensorIn(void)
{
    return (float)(temp_in/10.0f);
}

inline float RetSensorOut(void)
{
    return (float)(temp_out/10.0f);
}

inline void SensorInit(void){

    /* kontrola konfigurace senzoru */

    if(BCData.Sensors > SENSOR_OUT_IN) {
        BCData.Sensors = SENSOR_NO;
        StoreBCData();
    }
}

inline void SensorsSet(uint8_t set){

    BCData.Sensors = set;
    StoreBCData();
    ConvProgrs = 0;
    TimState = 0;
    temp_in = 0;
    temp_out = 0;

}

uint8_t SensorsGet(void) {
    return BCData.Sensors;
}

inline void SensorsPoll(void) {

    if(BCData.Sensors!=SENSOR_NO) {

        if(ConvProgrs==0){

            if(BCData.Sensors!=SENSOR_IN)  startT_out;
            if(BCData.Sensors!=SENSOR_OUT) startT_in;

            TimState = RetWdTimer();

            ConvProgrs = 1;
        }
        else {
            if (CompWdTimer(TimState,4)) { // cca 1 s

                if(BCData.Sensors!=SENSOR_IN)  readT_out(&temp_out);
                if(BCData.Sensors!=SENSOR_OUT) readT_in(&temp_in) ;

                ConvProgrs = 0;

            }
        }
    }
}
