#ifndef VARIABLES_H
#define VARIABLES_H

struct  {
    bool car_lock_close_pulse; //
    bool car_lock_open_pulse; //
    bool ignition_status; //
    bool shock_sensor_status; //
    bool l4949_power_sens_status; //
    bool standby_output_status; //
    bool recorder_output_status; //
    float voltage; //
}NodeData;

bool status_led_is_busy; //
bool flag_TimerCarToSleep_started_1; //
bool flag_TimerCarToSleep_started_2; //
bool flag_TimerRecorderTempOn_started; //
bool flag_car_is_closed; //


#endif

