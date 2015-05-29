/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#include "temp.h"

int16_t temp_in=0;
int16_t temp_out=0;

uint8_t Sensors = 0;
uint8_t ConvProgrs = 0;
uint8_t TimState = 0;

inline float RetSensorIn(void){
	return (float)(temp_in/10.0f);
}

inline float RetSensorOut(void){
	return (float)(temp_out/10.0f);
}

inline void SensorInit(void){

    /* nactem konfiguraci senzoru */
    Sensors = eeprom_read_byte(&eSensors);

    if(Sensors > SENSOR_OUT_IN) {
        Sensors = SENSOR_NO;
        eeprom_write_byte(&eSensors,Sensors);

    }
}

inline void SensorsSet(uint8_t set){

    Sensors = set;
    eeprom_write_byte(&eSensors,Sensors);
    ConvProgrs = 0;
    TimState = 0;
    temp_in = 0;
    temp_out = 0;

}

uint8_t SensorsGet(void){

    return Sensors;
}



inline void SensorsPoll(void){

    if(Sensors!=SENSOR_NO){

    if(ConvProgrs==0){

      if(Sensors!=SENSOR_IN)  startT_out;
      if(Sensors!=SENSOR_OUT)  startT_in;

        TimState = RetWdTimer();

        ConvProgrs = 1;
    }else {


        if (CompWdTimer(TimState,4)) {

            if(Sensors!=SENSOR_IN)  readT_out(&temp_out);
            if(Sensors!=SENSOR_OUT) readT_in(&temp_in) ;

           ConvProgrs = 0;

        }

    }

    }

}
