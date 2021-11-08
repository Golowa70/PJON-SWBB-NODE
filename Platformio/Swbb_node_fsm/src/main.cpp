//includes
  #include <Arduino.h>
  #include "setup_functions.h"
  #include "variables.h"

  #include "GyverButton.h"
  // #include "GyverHacks.h"
  #include "GyverTimer.h"
  #include "GyverFilters.h"
  #include <GyverWDT.h> 
  #include <EEPROMex.h>
  #include <EEPROMVar.h>
  #define PJON_INCLUDE_SWBB
  #include <PJON.h>
  #include <jled.h>
  #include "LowPower.h"
  #include "PinChangeInterrupt.h"
  #include "myFSM.h"
  #include "myFSM2.h"


GButton input1_close_pulse(INPUT_1_CLOSED_PULSE);
GButton input2_open_pulse(INPUT_2_OPENED_PULSE);
GButton input3_ignition_on(INPUT_3_IGNITION_ON );
GButton input4_shock_sensor(INPUT_4_SHOCK_SENSOR);

JLed status_led = JLed(STATUS_LED_PIN );
GTimer TimerIndicationsUpdate;
GTimer TimerCarToSleep_1;
GTimer TimerCarToSleep_2;
GTimer TimerRecorderTempOn;
GTimer TimerGotoSleep;
PJON<SoftwareBitBang> bus(PJON_BUS_PIN);

#if (DEBUG==1)
GTimer TimerDebugPrintUpdate;
#endif

GFilterRA voltage_filter;     // создать фильтр измерений напряжения 

void fnInputsUpdate(void);
void fnOutputsUpdate(void);
void fnPjonReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info); //
void fnStByOutputControl(void);
void fnIndicatorLed(uint8_t mode);
void fnWakeUpFromOpen(void);
void fnWakeUpFromClose(void);
void fnGotoSleep(void);
void fnRecorderOutputControl(void);
void fnWakeUpFromShockSensor(void);
void fnWakeUpFromIgnition(void);
enum fsm_event fnFsm1GetEvent(void);
enum fsm_2_event fnFsm2GetEvent(void);




void setup() {
  fnSetupIO();

  analogReference(INTERNAL);       // внутренний исочник опорного напряжения 1.1в
  voltage_filter.setCoef(0.1);    // установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
  voltage_filter.setStep(10);     // установка шага фильтрации (мс). Чем меньше, тем резче фильтр

  input1_close_pulse.setDebounce(50);         // настройка антидребезга ( 50 мс)
  input1_close_pulse.setTimeout(300);         // настройка таймаута на удержание ( 300 мс)
  input1_close_pulse.setClickTimeout(600);    // настройка таймаута между кликами (600 мс)
  input1_close_pulse.setType(HIGH_PULL);       // LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  input1_close_pulse.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка

  input2_open_pulse.setDebounce(50);        // 
  input2_open_pulse.setTimeout(300);        // 
  input2_open_pulse.setClickTimeout(600);   // 
  input2_open_pulse.setType(HIGH_PULL);
  input2_open_pulse.setDirection(NORM_OPEN);

  input3_ignition_on.setDebounce(50);        // 
  input3_ignition_on.setTimeout(300);        // 
  input3_ignition_on.setClickTimeout(600);   // 
  input3_ignition_on.setType(HIGH_PULL);
  input3_ignition_on.setDirection(NORM_OPEN);

  input4_shock_sensor.setDebounce(50);        // 
  input4_shock_sensor.setTimeout(300);        // 
  input4_shock_sensor.setClickTimeout(600);   // 
  input4_shock_sensor.setType(HIGH_PULL);
  input4_shock_sensor.setDirection(NORM_OPEN);

  TimerIndicationsUpdate.setInterval(2000); //
  TimerCarToSleep_1.setMode(AUTO);          //установка типа работы: AUTO или MANUAL (MANUAL нужно вручную сбрасывать reset)
  TimerCarToSleep_2.setMode(AUTO);
  TimerRecorderTempOn.setMode(AUTO);
  TimerGotoSleep.setMode(AUTO);
  TimerGotoSleep.setTimeout(1000);
  TimerGotoSleep.start();

  bus.strategy.set_pin(PJON_BUS_PIN);
  bus.begin();
  bus.set_receiver(fnPjonReceiver);

  Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с 

  attachInterrupt(0, fnWakeUpFromClose, FALLING); //
  attachInterrupt(1, fnWakeUpFromOpen, FALLING); //
  attachPCINT(digitalPinToPCINT(INPUT_3_IGNITION_ON), fnWakeUpFromIgnition, FALLING);
  //attachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR), fnWakeUpFromShockSensor, FALLING);

  
  

  #if(DEBUG==1)
  TimerDebugPrintUpdate.setInterval(1000);
  Serial.begin(9600);
  #endif

}

void loop() {
  // 
  Watchdog.reset();  // Переодический сброс watchdog, означающий, что устройство не зависло

  input1_close_pulse.tick(); // опрос входов с антидребезгом
  input2_open_pulse.tick();
  input3_ignition_on.tick();
  input4_shock_sensor.tick();

  fnInputsUpdate(); // обновление состояния входов

  status_led_is_busy = status_led.Update(); // обновление функции индикации светодиодом

  current_fsm_event = fnFsm1GetEvent();  // отслеживаем события для автомата FSM 1
  fnStByOutputControl();                 // FSM 1 автомат обработки выхода StBy

  current_fsm_2_event = fnFsm2GetEvent(); // отслеживаем события для автомата FSM 2
  fnRecorderOutputControl();              // FSM 2 автомат обработки выхода управления регистратором

  bus.receive(1000);                      // приём данных по протоколу PJON

  

  if(NodeData.ignition_status) fnIndicatorLed(1);   // если зажигание включено LED горит постоянно
    else {
      if(TimerCarToSleep_1.isEnabled()) fnIndicatorLed(2); //
        else{
          if(TimerCarToSleep_2.isEnabled()) fnIndicatorLed(3); //)
            else{
              if(TimerRecorderTempOn.isEnabled()) fnIndicatorLed(4); //
                else{
                  fnIndicatorLed(0);
              }
          }    
        }

    }

 

  fnOutputsUpdate(); //

  //delay(5000);

  

} // end loop


//--------------- Inputs -----------------
 void fnInputsUpdate(void){


  if (input1_close_pulse.state()) {
    NodeData.flag_car_is_closed = HIGH;  
    NodeData.car_lock_close_pulse = HIGH; //
    attachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR), fnWakeUpFromShockSensor, FALLING);
  }

  if (input2_open_pulse.isPress()){

    NodeData.flag_car_is_closed = LOW;  
    NodeData.car_lock_open_pulse = HIGH; //
    detachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR));
  }


  if (input3_ignition_on.isHold()) {
    NodeData.ignition_status = HIGH;
    NodeData.flag_car_is_closed = LOW;  //   ??????
    detachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR));
  }
  else NodeData.ignition_status = LOW; //

  if (input4_shock_sensor.state()) NodeData.shock_sensor_status = HIGH; //
  else NodeData.shock_sensor_status = LOW; //    
                                                                                                                                                                      
 }

//--------------- Outputs -----------------
  void fnOutputsUpdate(void){

   digitalWrite(OUTPUT_1_STANDBY_ON, NodeData.standby_output_status); //
   digitalWrite(OUTPUT_2_RECORDER_ON, NodeData.recorder_output_status); //

  }

//----------- Status led --------------------

  void fnIndicatorLed(uint8_t mode){

    switch (mode)
    {
    case 1 :
            status_led = JLed(STATUS_LED_PIN).On(); //
            break;

    case 2 :
            if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(500,500).Repeat(2); //
            break;

    case 3 :
            if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(250,250).Repeat(2); //
            break;    

    case 4 :  
            if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(100,100).Repeat(2); //
            break;              

    default:
            status_led = JLed(STATUS_LED_PIN).Off(); //
            break;

    }
    

  }

//---------- Pjon receiver ------------------

  void fnPjonReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
    
    if(payload[0] == 'R') {

      bus.reply( &NodeData, sizeof(NodeData));

      #if(DEBUG==1)
      Serial.print("Принят запрос от ");
      Serial.println(packet_info.receiver_id); //
      Serial.println();
      #endif
      
    }                                                                                              
  }

//---------- Stand-By output control FSM 1 --------------------------
  
  
  void fnStByOutputControl(void){
     

    switch (current_fsm_state){
         
      case stby_out_is_off :
                            switch (current_fsm_event) {
                            
                              case ignition_turned_on :
                                                      NodeData.standby_output_status = HIGH;
                                                      TimerCarToSleep_1.reset();
                                                      TimerCarToSleep_1.stop();
                                                      digitalWrite(STATUS_LED_PIN, HIGH); //
                                                      current_fsm_state = stby_out_is_on; //
                                                      break;

                              case car_turned_open : 
                                                    NodeData.standby_output_status = HIGH;
                                                    digitalWrite(STATUS_LED_PIN, HIGH); //
                                                    current_fsm_state = stby_out_is_on; //
                                                    TimerCarToSleep_1.reset();
                                                    TimerCarToSleep_1.setTimeout(DELAY_CAR_TO_SLEEP_1);
                                                    TimerCarToSleep_1.start(); 
                                                    break;

                              case car_turned_close : 
                                                    if(!NodeData.recorder_output_status){
                                                      TimerCarToSleep_2.setTimeout(DELAY_CAR_TO_SLEEP_2);
                                                      TimerCarToSleep_2.start();                                                     
                                                    }
                                                    break;

                              case timer_2_ready  :                                                     
                                                    TimerCarToSleep_2.reset();
                                                    TimerCarToSleep_2.stop();  
                                                   // flag_goto_sleep = 1;                                                    
                                                    fnGotoSleep();
                                                    break;               

                              default:
                                      break;
                            }

                            break;
        
      case stby_out_is_on  :
                            switch (current_fsm_event){
                              
                              case ignition_turned_off:  
                                                      if(!TimerCarToSleep_1.isEnabled()){
                                                        TimerCarToSleep_1.reset();
                                                        TimerCarToSleep_1.setTimeout(DELAY_CAR_TO_SLEEP_1);
                                                        TimerCarToSleep_1.start(); 
                                                      }
                                                      break;

                              case ignition_turned_on :
                                                      NodeData.standby_output_status = HIGH;
                                                      TimerCarToSleep_1.reset();
                                                      TimerCarToSleep_1.stop();
                                                      digitalWrite(STATUS_LED_PIN, HIGH); //
                                                      current_fsm_state = stby_out_is_on; //
                                                      break;                        
                              
                              case timer_1_ready    :
                                                      NodeData.standby_output_status = LOW;
                                                      digitalWrite(STATUS_LED_PIN, LOW);
                                                      TimerCarToSleep_1.reset();
                                                      TimerCarToSleep_1.stop();
                                                      current_fsm_state = stby_out_is_off;
                                                     // flag_goto_sleep = 1; 
                                                      fnGotoSleep();
                                                      break;

                              case timer_2_ready    :
                                                      NodeData.standby_output_status = LOW;
                                                      digitalWrite(STATUS_LED_PIN, LOW);
                                                      TimerCarToSleep_2.reset();
                                                      TimerCarToSleep_2.stop();
                                                      current_fsm_state = stby_out_is_off;
                                                     // flag_goto_sleep = 1; 
                                                      fnGotoSleep();
                                                      break;                        

                              case car_turned_close : 
                                                      if(!TimerCarToSleep_2.isEnabled()){
                                                        TimerCarToSleep_2.reset();
                                                        TimerCarToSleep_2.setTimeout(DELAY_CAR_TO_SLEEP_2);
                                                        TimerCarToSleep_2.start(); 

                                                        TimerCarToSleep_1.reset();
                                                        TimerCarToSleep_1.stop();
                                                      }
                                                      break; 

                              case car_turned_open  :                                                   
                                                        TimerCarToSleep_1.reset();
                                                        TimerCarToSleep_1.setTimeout(DELAY_CAR_TO_SLEEP_1);
                                                        TimerCarToSleep_1.start(); 
                                                    
                                                      break; 

                            }


      default:
              break;

    }

  }  

//---------- Recorder output control FSM 2 -----------------

  void fnRecorderOutputControl(void){

    switch (current_fsm_2_state ){
         
      case recorder_out_is_off :

                      switch (current_fsm_2_event) {
                      
                            case ignition_is_off :
                                                    
                                                  break;

                            case ignition_is_on:  
                                                  NodeData.recorder_output_status = HIGH;
                                                  current_fsm_2_state = recorder_out_is_on;
                                                  break;
                                
                            case car_is_closed :
                                                  break;

                            case car_is_opened : 
                                                  break;
                                  
                            case shock_sensor_is_active : 

                                                  if(NodeData.flag_car_is_closed) {
                                                    NodeData.recorder_output_status = HIGH;
                                                    current_fsm_2_state = recorder_out_is_on;
                                                  }
                                                  break;
                                
                            case  shock_sensor_is_not_active :
                                                  break;

                            case timer_3_ready : 
                                                  break;



                            default:
                                    break;
                      }

                  break;
        
      case recorder_out_is_on  :

                            switch (current_fsm_2_event) {
                      
                                  case ignition_is_off :
                                                          if(!NodeData.flag_car_is_closed){
                                                            NodeData.recorder_output_status = LOW;
                                                            current_fsm_2_state = recorder_out_is_off;
                                                          } 
                                                          break;

                                  case ignition_is_on:  
                                                           break;
                                      
                                  case car_is_closed :
                                                           break;

                                  case car_is_opened : 
                                                           break;
                                        
                                  case shock_sensor_is_active : 
                                                          if(NodeData.flag_car_is_closed) {
                                                            if(TimerRecorderTempOn.isEnabled()){
                                                              TimerRecorderTempOn.reset();
                                                            } 
                                                          }
                                                          break;
                                      
                                  case  shock_sensor_is_not_active :

                                                          if(NodeData.flag_car_is_closed){
                                                            if(!TimerRecorderTempOn.isEnabled()){
                                                              TimerRecorderTempOn.reset();
                                                              TimerRecorderTempOn.setTimeout(TIMEOUT_RECORDER_ON); 
                                                              TimerRecorderTempOn.start();
                                                            } 
                                                          }
                                                          break;

                                  case timer_3_ready : 
                                                          if(!NodeData.ignition_status){
                                                            NodeData.recorder_output_status = LOW;
                                                            current_fsm_2_state = recorder_out_is_off;
                                                            //flag_goto_sleep = 1;
                                                            fnGotoSleep();
                                                          }
                                                          break;



                                  default:
                                          break;
                      }


      default:
              break;

    }


  }  
     

//------------ wake Up from open -------------------

  void fnWakeUpFromOpen(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с
    TimerGotoSleep.setTimeout(TIMEOUT_MINIM_TO_SLEEP);
    TimerGotoSleep.start();
    current_fsm_event = car_turned_open;
  }

//------------ wake Up from close -------------------

  void fnWakeUpFromClose(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с
    TimerGotoSleep.setTimeout(TIMEOUT_MINIM_TO_SLEEP);
    TimerGotoSleep.start();     
  }

//------------ Wake Up from shock sensor -------------------

  void fnWakeUpFromShockSensor(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4c  
    TimerGotoSleep.setTimeout(TIMEOUT_MINIM_TO_SLEEP);
    TimerGotoSleep.start();
    current_fsm_2_event = shock_sensor_is_active;
  }
 

//----------------- Wake Up from ignition switch --------------------
  
  void fnWakeUpFromIgnition(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4c 
    TimerGotoSleep.setTimeout(TIMEOUT_MINIM_TO_SLEEP);
    TimerGotoSleep.start();
  }

//------------ goto sleep ----------------

  void fnGotoSleep(void){

    current_fsm_event = fsm1_no_events;
    current_fsm_2_event = fsm2_no_events;
    
    if(!NodeData.standby_output_status && !NodeData.recorder_output_status){

      if(TimerGotoSleep.isReady() ){

        digitalWrite(STATUS_LED_PIN, LOW); //
        fnOutputsUpdate();  //
        Watchdog.disable(); //
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //
      }
    }
  }

//--------- fsm1 get event ---------------

  enum fsm_event fnFsm1GetEvent(void){     //

    static enum fsm_event event;

    switch (fsm_counter)  {    //поочередно проверяем состояние входов, внутренних состояний и формируем событие для автомата

          case 0:
                if(NodeData.ignition_status != ignition_old_status){
                  if(!NodeData.ignition_status)event = ignition_turned_off;
                  if(NodeData.ignition_status)event = ignition_turned_on;
                  ignition_old_status = NodeData.ignition_status;
                } 
                fsm_counter++;
                break;

          case 1:
                if(TimerCarToSleep_1.isReady())event = timer_1_ready;
                fsm_counter++;
                break;

          case 2:
                if(TimerCarToSleep_2.isReady())event = timer_2_ready;
                fsm_counter++;
                break;

          case 3: 
                if(NodeData.car_lock_close_pulse)event = car_turned_close; 
                NodeData.car_lock_close_pulse = LOW; //                
                fsm_counter++;
                break;

          case 4: 
                if(NodeData.car_lock_open_pulse)event = car_turned_open;  
                NodeData.car_lock_open_pulse = LOW; //               
                fsm_counter++;
                break;    
          
          default:
                  fsm_counter = 0;
                  event = fsm1_no_events;
                  break;
    }

        

    #if(DEBUG_FSM_1 == 1) //отладка


      if(event != event_old_state){

        event_old_state = event;

        if(event == ignition_turned_off)event_counter_1++;
        if(event == ignition_turned_on)event_counter_2++;
        if(event == timer_1_ready)event_counter_3++;
        if(event == timer_2_ready)event_counter_4++;
        if(event == car_turned_close)event_counter_5++;
        if(event == car_turned_open)event_counter_6++;
      }


      if(TimerDebugPrintUpdate.isReady()){

        Serial.println("----------------- FSM 1 DEBUG SECTION ---------------");
        Serial.print("ignition_turned_off_cnt -  ");
        Serial.print(event_counter_1); //
        Serial.println();

        Serial.print("ignition_turned_on_cnt -  ");
        Serial.print(event_counter_2); //
        Serial.println();

        Serial.print("timer_1_ready_cnt -  ");
        Serial.print(event_counter_3); //
        Serial.println();

        Serial.print("timer_2_ready_cnt -  ");
        Serial.print(event_counter_4); //
        Serial.println();

        Serial.print("car_turned_close_cnt -  ");
        Serial.print(event_counter_5); //
        Serial.println();

        Serial.print("car_turned_open_cnt -  ");
        Serial.print(event_counter_6); //
        Serial.println();
        Serial.println();

        Serial.print("Текущее состояние FSM -  ");
        Serial.print(current_fsm_state); //
        Serial.println();

        Serial.print("Текущее событие FSM -  ");
        Serial.println(event); //
        Serial.println();
        Serial.println();
      }
     
    #endif


    return event;

  }

//--------------------- fsm2 get event ----------------------

  enum fsm_2_event fnFsm2GetEvent(void){

    static enum fsm_2_event event;

    switch(fsm_2_counter) {    // поочередно проверяем состояние входов, внутренних состояний и формируем событие для автомата

              case 0:
                    if(!NodeData.ignition_status) event = ignition_is_off;
                    if(NodeData.ignition_status) event = ignition_is_on;
                    fsm_2_counter++;
                    break;

              case 1:
                    if(NodeData.flag_car_is_closed) event = car_is_closed;
                    if(!NodeData.flag_car_is_closed) event = car_is_opened;
                    fsm_2_counter++;
                    break;

               case 2:
                    if(NodeData.shock_sensor_status) event = shock_sensor_is_active;
                    if(!NodeData.shock_sensor_status) event = shock_sensor_is_not_active;
                    fsm_2_counter++;
                    break;

              case 3:
                    if(TimerRecorderTempOn.isReady())event = timer_3_ready;
                    fsm_2_counter++;
                    break;             

              default :
                      fsm_2_counter = 0;
                      event = fsm2_no_events;
                      break;
    }


    #if(DEBUG_FSM_2 == 1)  

      if(event != fsm_2_event_old_state){

        fsm_2_event_old_state = event;

        if(event == ignition_is_off) fsm_2_event_counter_1++;
        if(event == ignition_is_on) fsm_2_event_counter_2++;
        if(event == car_is_closed) fsm_2_event_counter_3++;
        if(event == car_is_opened) fsm_2_event_counter_4++;
        if(event == shock_sensor_is_active) fsm_2_event_counter_5++;
        if(event == shock_sensor_is_not_active) fsm_2_event_counter_6++;
        if(event == timer_3_ready) fsm_2_event_counter_7++;
      }


      if(TimerDebugPrintUpdate.isReady()){
        
        Serial.println("----------------- FSM 2 DEBUG SECTION ---------------");
        Serial.print("ignition_is_off_cnt -  ");
        Serial.print(fsm_2_event_counter_1); //
        Serial.println();

        Serial.print("ignition_is_on_cnt -  ");
        Serial.print(fsm_2_event_counter_2); //
        Serial.println();

        Serial.print("car_is_closed_cnt -  ");
        Serial.print(fsm_2_event_counter_3); //
        Serial.println();

        Serial.print("car_is_opened_cnt -  ");
        Serial.print(fsm_2_event_counter_4); //
        Serial.println();

        Serial.print("shock_sensor_status -  ");
        Serial.print(NodeData.shock_sensor_status); //
        Serial.println();

        Serial.print("shock_sensor_is_not_active_cnt -  ");
        Serial.print(fsm_2_event_counter_6); //
        Serial.println();

        Serial.print("cnt -  ");
        Serial.print(fsm_2_event_counter_7); //
        Serial.println();

        Serial.print("Текущее состояние FSM_2 -  ");
        Serial.print(current_fsm_2_state); //
        Serial.println();

        Serial.print("Текущее событие FSM_2 -  ");
        Serial.println(event); //
        Serial.println();
        Serial.println();
      }

    #endif


    return event; // возвращаем текущее событие

  }
