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
#include <util/delay.h>

volatile uint32_t Counter1Ticks=0;

/* T0 overflow */
volatile uint8_t Counter0Val = 0;
volatile uint8_t Counter0_OVF = 0;

volatile uint8_t int0_flag=0;

cm_t CarData;
cm_t EEMEM eCarVals;

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
    OCR2A = 79; /* 100kHz */

    TCCR2A = 1<< COM2B0 | 1<<WGM21;

    TCCR2B = 1 << CS20;

    /* obcas ma timer tendenci nabehnout na spatne frekvenci - zejmena pokud doslo k suspendu a naslednemu odpojeni/pripojeni napajeni - snad vyreseno */

}

void InitCMeas(void)
{

    if(eeprom_read_byte(&eCarVals.init)!=EE_INIT_B)
    {

        waitms(200);

        if(eeprom_read_byte(&eCarVals.init)!=EE_INIT_B)
        {

            uint8_t i;

            CarData.init = EE_INIT_B;

            CarData.AvgInj=0;

            CarData.TripInj=0;

            CarData.TotInj=0;


            CarData.AvgImp = 0;
            CarData.TripImp =0;
            CarData.TotImp = 0;

            CarData.CalibMeters = AROSA_IMP;
            CarData.CalibInjFlow = AROSA_INJ;

            CarData.NumV = AROSA_V;

            CarData.TankL = 0;
            CarData.CalibL2 = 0;
            CarData.CalibL1 = 0;

            CarData.CalibV1 =0;
            CarData.CalibV2=0;

            for(i=0; i<CALCT_CNT; i++)
            {

                CarData.TankCalib[i].init = 0;
                CarData.TankCalib[i].mvolt = 0;
                CarData.TankCalib[i].litr = 0;
            }

            CarData.TankMode = TANK_MODE_STD;

            CarData.calib_volt = 999;

            StoreCMeas();
        }

    }

    eeprom_read_block(&CarData,&eCarVals,sizeof(cm_t));

    Init50kPD3();

    TCNT1=0;
    TCCR1B = 1<< CS11 | 1 << CS12;      /* counter 1 16bit PD5*/

    DDRD &= ~(1<<PD4);      /* input T0 */
    PORTD |= 1 << PD4;      /* pullup T0 */

    TIMSK1 = 1<<TOIE1; /* fixed missing INT initialization 29.7.2014 */

    TCNT0=0;
    TCCR0B=1<<CS02|1<<CS01|1<<CS00;    /* rissing edge T0*/

    TIMSK0 = 1 << TOIE0;        /* TIMER0_OVF_vect */

    if(CarData.TankMode == TANK_MODE_COMP) initComparator();
    else disComparator();

    /* osetrime narychlo chybnou hodnotu po prehravani firmware a zmene eeprom */
    if(getSleepTime() > 60 || getSleepTime() ==0) setSleepTime(30);

    if(getMenu1() > 5) setMenu1(4);


    sei();

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

        StartAD(V12_CH)   ;
        ReadAD(V12_CH);

        StartAD(V12_CH);
        V12 =ReadAD(V12_CH);

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

volatile uint8_t running = 0;


/* volana z interruptu casovacce */
void CMeasPoll(void)
{
    // uint32_t ticks=0;
    uint16_t imp=0;

    Counter1Ticks = TCNT1;
    Counter1Ticks+= (uint32_t)Tim1_OVF*65536;
    Counter1Ticks/=2;
    TCNT1 = 0;
    Tim1_OVF=0;
    Counter0Val = TCNT0;
    TCNT0=0;
    Counter0_OVF=Tim0_OVF;
    Tim0_OVF=0;

    /* osetreni maxima - pri vypnutem motoru, nektere vstrikovace spinaji vstupni logiku do log 0 a nechtelo to bez tohoto uspavat */
    if(  (Counter0Val || Counter0_OVF) || (Counter1Ticks<TICKS_MAX && Counter1Ticks!=0)  )
    {
        //ticks = Counter1Ticks;
        //ticks += (uint32_t)Counter1_OVF*65536;
        //ticks/=2;   /* delicka z 100 kHz na 50 kHz */

        CarData.AvgInj+= Counter1Ticks;
        CarData.TripInj += Counter1Ticks;
        CarData.TotInj  += Counter1Ticks;

        if(Counter0_OVF) imp = ((uint16_t)256 * (uint16_t)Counter0_OVF);

        CarData.AvgImp += (uint16_t)Counter0Val + imp;
        CarData.TripImp += (uint16_t)Counter0Val + imp;
        CarData.TotImp += (uint16_t)Counter0Val + imp;

        running=1;

    }
    else
    {
        running=0;
        Counter1Ticks=0;
        Counter0Val=0;
        Counter0_OVF=0;
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
    return ((mv*(uint32_t)(CarData.calib_volt+V12_R2))/V12_R2);

}
/* pridano 8.10.2011 - implementace po castech linearni aproximace nadrze */
tank_t GetTankL(void)
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

    if(k<0) k = 0;

    else if(k%10 > 4) k+=10;

    k/=10;

    return  (k>CarData.TankL?CarData.TankL:k);


}

uint16_t GetMaxDis(void)
{

    fuel_t f= GetAvgFuelCons();

    if(!f) return 0;

    uint16_t dis = ((uint16_t)GetTankL() * (uint16_t)100)/GetAvgFuelCons();

    return (dis*10);

}

fuel_t GetFuel(inj_t *inj)
{

    if(*inj ==0 || CarData.CalibInjFlow ==0 || CarData.NumV ==0) return 0;

    /* CalibInjFlow format 3500 = 350,0 ccm/min
        NumV 4 = 4 valce
        inj 50 000 = 1s

    */


    return (fuel_t)( (inj_t)(((inj_t)*inj/(inj_t)60)*(uint64_t)CarData.NumV*(uint64_t)CarData.CalibInjFlow)/((inj_t)50000000UL) );
    //inj_t calc1 = (( (inj_t)(*inj/6000)*(inj_t)CarData.NumV*(inj_t)CarData.CalibInjFlow)/(500000) );

//       return calc1;

    // return ( ((inj_t)(*inj/60000)*(inj_t)CarData.NumV*(inj_t)CarData.CalibInjFlow)/(50000) );


    /*inj_t calc1 = ( ((uint32_t)(*inj_imp/6000)*(uint32_t)CarData.NumV*(uint32_t)CarData.CalibInjFlow)/(500000) );

    inj_t calc;

    calc = * inj_min*CarData.CalibInjFlow*CarData.NumV;

    if(calc > *inj_min) calc /=1000;
    else calc = ((*inj_min/1000)*CarData.NumV*CarData.CalibInjFlow);

    return (calc+calc1); */
}

inline fuel_t GetCurrFuel(void)
{

    return GetFuel(&CarData.AvgInj);

}

inline fuel_t GetTripFuel(void)
{

    return GetFuel(&CarData.TripInj);

}

inline void StoreCMeas(void)
{

    eeprom_update_block(&CarData,&eCarVals,sizeof(cm_t));


}

m_t RetMeters(odo_t * imp)
{

    if(CarData.CalibMeters ==0 || *imp ==0 ) return 0;

    m_t m=0;

    /*if( (*imp)>((odo_t)CarData.CalibMeters*(odo_t)1000))    return (    ((*imp) / (odo_t)CarData.CalibMeters)* (odo_t)100);

    else return (    ((*imp) * (odo_t)100)  / (odo_t)CarData.CalibMeters);*/
    if(*imp < 42000000UL)
    {

        m=(((*imp) * (odo_t)100)  / (odo_t)CarData.CalibMeters);

    }
    else
    {

        m=(    ((*imp) / (odo_t)CarData.CalibMeters)* (odo_t)100);

    }
    return m;

}

fuel_t RetFuelCons(odo_t * imp, inj_t *inj)
{



    if( *imp<4   || *inj ==0 || CarData.CalibInjFlow ==0 || CarData.NumV ==0) return 0;

    /* CalibInjFlow format 3500 = 350,0 ccm/min
       NumV 4 = 4 valce
       inj 50 000 = 1s

        */

    /* uprava nejspise neco na zpusob ukladani minut v 32bit promenne + druhy 32bit prom na zbytky*/
    return ((*inj * (uint64_t)CarData.NumV * (uint64_t)CarData.CalibInjFlow)/(uint64_t)RetMeters(imp))/(uint64_t)30000 ;

    /*    inj_t calc = 0;
        inj_t calc1=0;

        odo_t meters = RetMeters(imp); */

    /*  if(*inj_imp){

          if(*inj_imp < 300000UL){

              calc = (((*inj_imp* CarData.CalibInjFlow)/meters)*CarData.NumV)/30000;

          }else {


              calc = (((*inj_imp * CarData.NumV)/10000)*CarData.CalibInjFlow)/meters/3;

          }

      }

      if(*inj_min){

          calc1 = *inj_min * CarData.CalibInjFlow*CarData.NumV;

          if(*inj_min < calc1){


              if(meters > 10000) {

                  calc1 = ((*inj_min*CarData.CalibInjFlow*CarData.NumV)/(meters/100));

              }else calc1 = ((*inj_min*CarData.CalibInjFlow*CarData.NumV)/meters)*100;


          }else {

              if(meters > 10000){

                  calc1 = (*inj_min/(meters/100))*CarData.CalibInjFlow*CarData.NumV;

              }else {

                  calc1 = ((*inj_min/meters)*100)*CarData.CalibInjFlow*CarData.NumV;

              }


          }
      }

      return (calc+calc1);*/

    /*   if(*inj_imp)
      {

          if(*inj_imp < 42000UL){

              calc = (  (( (*inj_imp* (inj_t)CarData.CalibInjFlow))*(inj_t)CarData.NumV) /meters)/(inj_t)3000;

          }else if(*inj_imp < 420000UL)
          {

              calc = (  (( (*inj_imp* (inj_t)CarData.CalibInjFlow)/(inj_t)10 )*(inj_t)CarData.NumV) /meters)/(inj_t)300;

          }
          else
          {
              calc = (((*inj_imp * (inj_t)CarData.NumV)/(inj_t)100)*(inj_t)CarData.CalibInjFlow)/meters/(inj_t)30;
          }


      }


      if(*inj_min)
      {

        if(*inj_min < 42UL){

              calc1 = (  *inj_min*(inj_t)CarData.CalibInjFlow*(inj_t)CarData.NumV*1000 ) /meters;

        }else if(*inj_min < 420UL){

              calc1 = (   *inj_min* (inj_t)CarData.CalibInjFlow*100*(inj_t)CarData.NumV) /(meters/10);

        }else if(*inj_min < 4200UL){
              calc1 = (   *inj_min*10* (inj_t)CarData.CalibInjFlow*(inj_t)CarData.NumV) /(meters/100);
        }else if(*inj_min < 42000UL){
              calc1 = (   (*inj_min)* (inj_t)CarData.CalibInjFlow*(inj_t)CarData.NumV) /(meters/1000);
        }
        else if(*inj_min < 420000UL){
              calc1 = (   (*inj_min/(inj_t)10)* (inj_t)CarData.CalibInjFlow*(inj_t)CarData.NumV) /(meters/10000);
        }
        else{
              calc1 = (  (( (*inj_min/(inj_t)100)* (inj_t)CarData.CalibInjFlow) )*(inj_t)CarData.NumV) /(meters/100000);
        }
      } */

//      return ((calc+calc1)/10);
}

fuel_t RetFuelConsL(void)
{



    if( Counter1Ticks == 0 || CarData.NumV ==0 || CarData.CalibInjFlow ==0) return 0;

    /* Counter / 50 000 -> (50khz) mam sekundy. *valce* InjFlow v (ccm/min)/10 / 60 / 1000 ( na dm2=litry) * 10 ( presnost na jedno misto) *3600 -> xx,x L za hodinu */

    return ( (((((uint64_t)Counter1Ticks *3* (uint64_t)CarData.NumV)/250) * (uint64_t)CarData.CalibInjFlow/10))/1000 );
}

inline fuel_t GetCurrFuelCons(void)
{

    inj_t ticks = Counter1Ticks;
    odo_t imp = GetImpCnt();


    return RetFuelCons(&imp,&ticks);

}

uint16_t GetInjCnt(void)
{

    return Counter1Ticks;

}

uint16_t GetImpCnt(void)
{

    return ((uint16_t)Counter0Val+((uint16_t)Counter0_OVF*(uint16_t)256));

}

inline fuel_t GetCurrFuelConsL(void)
{

    return RetFuelConsL();

}

inline fuel_t GetAvgFuelCons(void)
{

    return RetFuelCons(&CarData.AvgImp,&CarData.AvgInj);

}

inline fuel_t GetTripFuelCons(void)
{

    return RetFuelCons(&CarData.TripImp,&CarData.TripInj);

}

inline fuel_t GetTotFuelCons(void)
{

    return RetFuelCons(&CarData.TotImp,&CarData.TotInj);

}

inline m_t GetCurrMeters(void)
{

    return RetMeters(&CarData.AvgImp);

}

inline m_t GetTripMeters(void)
{

    return RetMeters(&CarData.TripImp);

}

inline m_t GetTotMeters(void)
{

    return RetMeters(&CarData.TotImp);

}

m_t GetActMeters(void)
{

    odo_t imp = GetImpCnt();

    return RetMeters(&imp);

}

inline uint8_t IsRunning(void)
{

    return running;

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

inline void SetCalibInjFlow(ccmmin_t ij)
{

    CarData.CalibInjFlow = ij;

}

inline uint8_t GetCalibValve(void)
{

    return CarData.NumV;

}
inline void SetCalibValve(uint8_t v)
{

    CarData.NumV = v;

}
/*
inline mv_t GetCalibV1(void){

return CarData.CalibV1;
}

inline mv_t GetCalibV2(void){

return CarData.CalibV2;
}

inline void SetCalibV1(mv_t v){

CarData.CalibV1 = v;

}

inline void SetCalibV2(mv_t v){

CarData.CalibV2 = v;

}

inline tank_t GetCalibL1(void){

return CarData.CalibL1;

}
*/
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
/*
inline tank_t GetCalibL2(void){

return CarData.CalibL2;

}

inline void SetCalibL1(tank_t l){

CarData.CalibL1 = l;

}

inline void SetCalibL2(tank_t l){

CarData.CalibL2 = l;

}
*/
inline tank_t GetCalibL(void)
{

    return CarData.TankL;

}
inline void SetCalibL(tank_t l)
{


    CarData.TankL = l;

}

inline void ClearCMeas(void)
{

    eeprom_update_byte(&eCarVals.init,0);

}
inline void ClearAvgMeas(void)
{
    cli();
    CarData.AvgInj = 0;
    CarData.AvgImp = 0;
    sei();

}
inline void ClearTripMeas(void)
{
    cli();
    CarData.TripInj = 0;
    CarData.TripImp = 0;
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

uint16_t GetActSpeed(void)
{
    return ( (uint32_t)GetImpCnt()*3600)/CarData.CalibMeters;
}

inline uint16_t GetCalibVolt(void)
{

    return CarData.calib_volt;
}

inline void SetCalibVolt(uint16_t calib)
{

    CarData.calib_volt = calib;
}

inline uint8_t getSleepTime(void)
{

    return CarData.SleepTime;
}

inline void setSleepTime(uint8_t time)
{

    CarData.SleepTime = time;
}

/* zobrazovaci workaround..kazdy chce neco jineho */

inline void setMenu1(uint8_t menu){
    CarData.menu1 = menu;
}

inline uint8_t getMenu1(void){
    return CarData.menu1;
}

// nove fce pro obsluhu sleepFlag

inline void setSleepFlag(void){
    eeprom_write_byte(&eCarVals.sleepFlag ,1);
}

inline void clearSleepFlag(void){
    eeprom_write_byte(&eCarVals.sleepFlag ,0);
}

inline uint8_t isSleeFlagEnabled(void){
    return eeprom_read_byte(&eCarVals.sleepFlag);
}
