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


GButton input1_close_pulse(INPUT_1_CLOSED_PULSE);
GButton input2_open_pulse(INPUT_2_OPENED_PULSE);
GButton input3_ignition_on(INPUT_3_IGNITION_ON );
GButton input4_shock_sensor(INPUT_4_SHOCK_SENSOR);

JLed status_led = JLed(STATUS_LED_PIN );
GTimer TimerIndicationsUpdate;
GTimer TimerCarToSleep_1;
GTimer TimerCarToSleep_2;
GTimer TimerRecorderTempOn;
PJON<SoftwareBitBang> bus(PJON_BUS_PIN);


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



void setup() {
  fnSetupIO();

  analogReference(INTERNAL);       // внутренний исочник опорного напряжения 1.1в
  voltage_filter.setCoef(0.1);    // установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
  voltage_filter.setStep(10);     // установка шага фильтрации (мс). Чем меньше, тем резче фильтр

  input1_close_pulse.setDebounce(50);         // настройка антидребезга (по умолчанию 80 мс)
  input1_close_pulse.setTimeout(300);         // настройка таймаута на удержание (по умолчанию 500 мс)
  input1_close_pulse.setClickTimeout(600);    // настройка таймаута между кликами (по умолчанию 300 мс)
  input1_close_pulse.setType(HIGH_PULL);       // LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  input1_close_pulse.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка

  input2_open_pulse.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)
  input2_open_pulse.setTimeout(300);        // настройка таймаута на удержание (по умолчанию 500 мс)
  input2_open_pulse.setClickTimeout(600);   // настройка таймаута между кликами (по умолчанию 300 мс)
  input2_open_pulse.setType(HIGH_PULL);
  input2_open_pulse.setDirection(NORM_OPEN);

  input3_ignition_on.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)
  input3_ignition_on.setTimeout(300);        // настройка таймаута на удержание (по умолчанию 500 мс)
  input3_ignition_on.setClickTimeout(600);   // настройка таймаута между кликами (по умолчанию 300 мс)
  input3_ignition_on.setType(HIGH_PULL);
  input3_ignition_on.setDirection(NORM_OPEN);

  input4_shock_sensor.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)
  input4_shock_sensor.setTimeout(300);        // настройка таймаута на удержание (по умолчанию 500 мс)
  input4_shock_sensor.setClickTimeout(600);   // настройка таймаута между кликами (по умолчанию 300 мс)
  input4_shock_sensor.setType(HIGH_PULL);
  input4_shock_sensor.setDirection(NORM_OPEN);

  TimerIndicationsUpdate.setInterval(2000); //
  TimerCarToSleep_1.setMode(AUTO); //установка типа работы: AUTO или MANUAL (MANUAL нужно вручную сбрасывать reset)
  TimerCarToSleep_2.setMode(AUTO);
  TimerRecorderTempOn.setMode(AUTO);

  bus.strategy.set_pin(PJON_BUS_PIN);
  bus.begin();
  bus.set_receiver(fnPjonReceiver);

  Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с 

  attachInterrupt(0, fnWakeUpFromClose, FALLING); //
  attachInterrupt(1, fnWakeUpFromOpen, FALLING); //
  //attachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR), fnWakeUpFromShockSensor, FALLING);
  attachPCINT(digitalPinToPCINT(INPUT_3_IGNITION_ON), fnWakeUpFromShockSensor, FALLING);

  #if(DEBUG==1)
  Serial.begin(9600);
  #endif

}

void loop() {
  // 
  Watchdog.reset();  // Переодический сброс watchdog, означающий, что устройство не зависло

  input1_close_pulse.tick(); 
  input2_open_pulse.tick();
  input3_ignition_on.tick();
  input4_shock_sensor.tick();

  fnInputsUpdate(); //
  fnStByOutputControl();  //
  fnRecorderOutputControl(); //

  //NodeData.car_lock_close_pulse = LOW; //
  //NodeData.car_lock_open_pulse = LOW; //

  bus.receive(1000); //

  status_led_is_busy = status_led.Update(); //
  
  if(TimerCarToSleep_1.isEnabled()) fnIndicatorLed(1); //
  if(TimerCarToSleep_2.isEnabled()) fnIndicatorLed(1); //
  if(TimerRecorderTempOn.isEnabled()) fnIndicatorLed(2); //
  //digitalWrite(STATUS_LED_PIN, flag_car_is_closed);

  fnOutputsUpdate(); //

  //delay(5000);

}


//--------------- Inputs -----------------
 void fnInputsUpdate(void){

  if (input1_close_pulse.isPress()) {

    NodeData.car_lock_close_pulse = HIGH; //
    attachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR), fnWakeUpFromShockSensor, FALLING);
  }

  if (input2_open_pulse.isPress()){

    NodeData.car_lock_open_pulse = HIGH; //
    detachPCINT(digitalPinToPCINT(INPUT_4_SHOCK_SENSOR));
  }

  if (input3_ignition_on.isHold()) NodeData.ignition_status = ON; //
  else NodeData.ignition_status = OFF; //

  if (input4_shock_sensor.isPress()) NodeData.shock_sensor_status = HIGH; //
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
            if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(500,500).Repeat(2); //
            break;

    case 2 :
            if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(200,200).Repeat(2); //
            break;

    default:
            break;

    }
    
    //if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(500,500).Repeat(2); //
    //if(!status_led_is_busy) status_led = JLed(STATUS_LED_PIN).Blink(500,500).Forever(); //

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

//---------- Stand-By output control --------------------------

  void fnStByOutputControl(void){

    if( NodeData.ignition_status ){

      flag_car_is_closed  = LOW;
      NodeData.standby_output_status = HIGH;
      TimerCarToSleep_1.reset();
      TimerCarToSleep_1.stop();
      flag_TimerCarToSleep_started_1= LOW;
      //detachInterrupt(0);
      digitalWrite(STATUS_LED_PIN, HIGH); //  
    }


    if(!NodeData.ignition_status  && NodeData.standby_output_status && !flag_TimerCarToSleep_started_1  ) { 

      TimerCarToSleep_1.reset();
      TimerCarToSleep_1.setTimeout(DELAY_CAR_TO_SLEEP_1);
      TimerCarToSleep_1.start(); 
      flag_TimerCarToSleep_started_1 = HIGH;        
    }


    if(TimerCarToSleep_1.isReady()){  // если таймер досчитал

      NodeData.standby_output_status = LOW;  // всё гасим
      TimerCarToSleep_1.stop();
      fnGotoSleep();                         // и идём спать
    }


    if(TimerCarToSleep_2.isReady()){  // если таймер досчитал

      NodeData.standby_output_status = LOW;  // всё гасим
      TimerCarToSleep_2.stop();
      TimerCarToSleep_1.stop();
      flag_TimerCarToSleep_started_2 = LOW;
      fnGotoSleep();                         // и идём спать
    }


    if(NodeData.car_lock_close_pulse){ 

      flag_car_is_closed = HIGH;           

      if(!NodeData.ignition_status && NodeData.standby_output_status && !flag_TimerCarToSleep_started_2){  

        TimerCarToSleep_2.reset();
        TimerCarToSleep_2.setTimeout(DELAY_CAR_TO_SLEEP_2);
        TimerCarToSleep_2.start(); 
        flag_TimerCarToSleep_started_2 = HIGH; 
      }         
    }


    if(NodeData.car_lock_open_pulse) {

      flag_car_is_closed = LOW;

      if(!NodeData.ignition_status){
        TimerCarToSleep_1.reset();
        TimerCarToSleep_1.setTimeout(DELAY_CAR_TO_SLEEP_1);
        TimerCarToSleep_1.start();  
        NodeData.standby_output_status = HIGH;   
      }    
    }
         
    NodeData.car_lock_close_pulse = LOW; //
    NodeData.car_lock_open_pulse = LOW; //

  }  

//------------- Recorder output control-----------------

  void fnRecorderOutputControl(void){

    if( NodeData.ignition_status && !flag_car_is_closed ){

      NodeData.recorder_output_status = HIGH;
      TimerRecorderTempOn.stop();
      flag_TimerRecorderTempOn_started = LOW; 
    }


    if(!NodeData.ignition_status && !flag_car_is_closed && !TimerRecorderTempOn.isEnabled()){

      NodeData.recorder_output_status = LOW;
    }


    if(NodeData.shock_sensor_status && flag_car_is_closed ) {

      TimerRecorderTempOn.reset();
      TimerRecorderTempOn.setTimeout(TIMEOUT_RECORDER_ON); 
      TimerRecorderTempOn.start();
      flag_TimerRecorderTempOn_started = HIGH;
      NodeData.recorder_output_status = HIGH;

    }


    if(TimerRecorderTempOn.isReady()) {

      NodeData.recorder_output_status = LOW;
      TimerRecorderTempOn.stop();
      flag_TimerRecorderTempOn_started = LOW; 
      if(flag_car_is_closed)fnGotoSleep();

    }

  }  
     
  

//------------ wake Up from open -------------------

  void fnWakeUpFromOpen(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с

  }

//------------ wake Up from close -------------------

  void fnWakeUpFromClose(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4с

  }

 //------------ Wake Up from shock sensor -------------------

  void fnWakeUpFromShockSensor(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4c   
    
  }
 

//----------------- Wake Up from ignition switch --------------------
  
  void fnWakeUpFromIgnition(void){

    Watchdog.enable(RESET_MODE, WDT_PRESCALER_512); // Режим сторжевого сброса , таймаут ~4c 

  }

//------------ goto sleep ----------------

  void fnGotoSleep(void){

    digitalWrite(STATUS_LED_PIN, LOW); //
    fnOutputsUpdate();  //
    Watchdog.disable(); //
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //
  }


