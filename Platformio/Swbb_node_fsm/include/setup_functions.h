#ifndef SETUP_FUNCTIONS_H
#define SETUP_FUNCTIONS_H

#include <Arduino.h>
#include "defines.h"


void fnSetupIO(void) {

    //inputs declaration
    pinMode(INPUT_1_CLOSED_PULSE, INPUT_PULLUP);  // импульс закрывания ЦЗ // поменять на без подтяжки !!!!!!
    pinMode(INPUT_2_OPENED_PULSE, INPUT_PULLUP);  // импульс открывания ЦЗ
    pinMode(INPUT_3_IGNITION_ON, INPUT_PULLUP);   // зажигание
    pinMode(INPUT_4_SHOCK_SENSOR, INPUT_PULLUP);  // вход от датчика удара
    pinMode(L4949_POWER_SENS, INPUT);             // вход от супервизора питания в составе L4949
    pinMode(BATTERY_VOLTAGE, INPUT);              // вход измерения напряжения бортсети

    pinMode(RS485_RX, INPUT);
    pinMode(RS485_TX, INPUT);
    pinMode(RS485_DE, INPUT);

    //outputs declaration
    pinMode(OUTPUT_1_STANDBY_ON, OUTPUT);  // выход StBy(управляет питанием всего доп. оборудования)
    pinMode(OUTPUT_2_RECORDER_ON, OUTPUT); // вуход управляет включением регистратора
    pinMode(STATUS_LED_PIN , OUTPUT);      // светодиод индикации состояния модуля


}

#endif