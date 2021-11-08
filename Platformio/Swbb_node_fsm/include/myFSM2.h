#ifndef MYFSM2_H
#define MYFSM2_H


enum fsm_2_event{

    fsm2_no_events, 
    ignition_is_off, ignition_is_on,
    car_is_closed, car_is_opened, 
    shock_sensor_is_active, shock_sensor_is_not_active,
    timer_3_ready,
    fsm2_max_events
    };

enum fsm_2_state {recorder_out_is_off, recorder_out_is_on, fsm2_max_states}; // перечисление возможных состояний автомата 2

enum fsm_2_event current_fsm_2_event = fsm2_no_events;
enum fsm_2_state current_fsm_2_state = recorder_out_is_off;  //начальное состояние автомата 2

unsigned int fsm_2_counter = 0;

    //отладка
    #if(DEBUG_FSM_2 == 1)

        uint16_t fsm_2_event_counter_1;
        uint16_t fsm_2_event_counter_2;
        uint16_t fsm_2_event_counter_3;
        uint16_t fsm_2_event_counter_4;
        uint16_t fsm_2_event_counter_5;
        uint16_t fsm_2_event_counter_6;
        uint16_t fsm_2_event_counter_7;

        enum fsm_2_event fsm_2_event_old_state;

    #endif

#endif