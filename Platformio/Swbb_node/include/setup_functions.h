#ifndef SETUP_FUNCTIONS_H
#define SETUP_FUNCTIONS_H

#include <Arduino.h>
#include "defines.h"


void fnSetupIO(void) {

  //inputs declaration
pinMode(INPUT_1_CLOSED_PULSE, INPUT_PULLUP);  // поменять на без подтяжки !!!!!!
pinMode(INPUT_2_OPENED_PULSE, INPUT_PULLUP);
pinMode(INPUT_3_IGNITION_ON, INPUT_PULLUP);
pinMode(INPUT_4_SHOCK_SENSOR, INPUT_PULLUP);
pinMode(L4949_POWER_SENS, INPUT);
pinMode(BATTERY_VOLTAGE, INPUT);

pinMode(RS485_RX, INPUT);
pinMode(RS485_TX, INPUT);
pinMode(RS485_DE, INPUT);

  //outputs declaration
pinMode(OUTPUT_1_STANDBY_ON, OUTPUT);
pinMode(OUTPUT_2_RECORDER_ON, OUTPUT);
pinMode(STATUS_LED_PIN , OUTPUT);


}

#endif