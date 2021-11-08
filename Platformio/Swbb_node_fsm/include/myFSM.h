#ifndef MYFSM_H
#define MYFSM_H


enum fsm_event   // перечисление возможных событий автоматов
{   fsm1_no_events, 
    ignition_turned_off, ignition_turned_on, 
    timer_1_ready, timer_2_ready,
    car_turned_close, car_turned_open,
    fsm1_max_events
};

enum fsm_state {stby_out_is_off, stby_out_is_on, fsm1_max_states}; // перечисление возможных состояний автомата 1

enum fsm_event current_fsm_event = fsm1_no_events; //
enum fsm_state current_fsm_state = stby_out_is_off;  //начальное состояние автомата 1

uint8_t fsm_counter = 0; // служебный счетчик в функции генерации события  fnFsmGetEvent().

    //отладка
    #if(DEBUG_FSM_1 == 1)

        uint16_t event_counter_1;
        uint16_t event_counter_2;
        uint16_t event_counter_3;
        uint16_t event_counter_4;
        uint16_t event_counter_5;
        uint16_t event_counter_6;
        
        enum fsm_event event_old_state;

    #endif


#endif