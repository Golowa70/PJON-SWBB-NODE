#ifndef VARIABLES_H
#define VARIABLES_H

struct  {
    bool car_lock_close_pulse; //
    bool car_lock_open_pulse; //
    bool flag_car_is_closed; //
    bool ignition_status; //
    bool shock_sensor_status; //
    bool l4949_power_sens_status; //
    bool standby_output_status; //
    bool recorder_output_status; //
    float voltage; //
}NodeData;

bool ignition_old_status;
bool shock_sensor_old_status;

bool status_led_is_busy; // функция индикации занята
bool flag_car_is_closed_old_state;
bool flag_goto_sleep;//


#endif

