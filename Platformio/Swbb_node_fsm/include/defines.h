#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

#define DEBUG       0 // включение/выключение отладки по UART(0-выкл, 1-вкл) 
#define DEBUG_FSM_1 0
#define DEBUG_FSM_2 0

#define INPUT_1_CLOSED_PULSE    2 //
#define INPUT_2_OPENED_PULSE    3 //
#define INPUT_3_IGNITION_ON    14 //A0  
#define INPUT_4_SHOCK_SENSOR   15 //A1
#define L4949_POWER_SENS       16 //A2
#define BATTERY_VOLTAGE        17 //A3

#define OUTPUT_1_STANDBY_ON     9
#define OUTPUT_2_RECORDER_ON   10
#define STATUS_LED_PIN         13

#define RS485_RX                4
#define RS485_TX                5
#define RS485_DE                6
#define PJON_BUS_PIN            7

#define DELAY_CAR_TO_SLEEP_1    15000   // таймаут ухода в сон после выключения зажигания
#define DELAY_CAR_TO_SLEEP_2     5000   // таймаут ухода в сон после закрытия авто
#define TIMEOUT_RECORDER_ON      8000   // таймаут включения камеры при срабатывании датчика удара 
#define TIMEOUT_MINIM_TO_SLEEP   1000   // минимальное время между выходом из сна и входом в следующий сон


#endif