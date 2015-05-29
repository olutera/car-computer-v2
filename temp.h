/****************************************************************************
*
* Authors:    Ondrej Lutera
* Copyright:  GPL
*
*
*****************************************************************************/
#ifndef TEMP_H_INCLUDED
#define TEMP_H_INCLUDED

//#include "ds18x20.h"
//#include "onewire.h"
//#include <avr/eeprom.h>
#include "ds18b20.h"
#include "wdt_timer.h"
#include <avr/eeprom.h>

#define SENSOR_NO 0
#define SENSOR_OUT 1
#define SENSOR_IN   2
#define SENSOR_OUT_IN   3

/* return sensors ids */
//uint8_t InitSensors(void);

/* swap sensors in<->out */
//void SwapSensors(void);

/* 10*120ms temp meas in main()*/
void SensorsPoll(void);

/* user funcs return temps */
float RetSensorIn(void);
float RetSensorOut(void);
void SensorsSet(uint8_t set);
uint8_t SensorsGet(void);
void SensorInit(void);

#endif // TEMP_H_INCLUDED
