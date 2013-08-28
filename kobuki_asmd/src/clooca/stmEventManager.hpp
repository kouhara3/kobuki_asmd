#pragma once

class STMEventManager{
public:
  enum Event{
/*    LEFT_BUMPER_PRESSED,
    LEFT_BUMPER_RELEASED,
    CENTER_BUMPER_PRESSED,
    CENTER_BUMPER_RELEASED,
    RIGHT_BUMPER_PRESSED,
    CENTER_BUMPER_RELEASED,
    LEFT_FAR_IR_FOUND,
    LEFT_FAR_IR_MISSED,
    LEFT_NEAR_IR_FOUND,
    LEFT_NEAR_IR_MISSED,
    CENTER_FAR_IR_FOUND,
    CENTER_FAR_IR_MISSED,
    CENTER_NEAR_IR_FOUND,
    CENTER_NEAR_IR_MISSED,
    RIGHT_FAR_IR_FOUND,
    RIGHT_FAR_IR_MISSED,
    RIGHT_NEAR_IR_FOUND,
    RIGHT_NEAR_IR_MISSED,
*/
    BUMPER_PRESSED = 0,
    BUMPER_RELEASED = -1,
    IR_FOUND = -1,
    IR_MISSED = -1,
    RUN_REACHED = 4,
    TURN_REACHED = 3,
  };

  STMEventManager(){
    last_bumper_signal = 0x00;
    last_ir_signal = 0x00;
    last_run_state = KobukiManager::STOP;
  }

  void checkEvent( KobukiStateMachine& stm ){

    kobuki::SensorManager::Data new_sensor = stm.map.getSensorData();
    kobuki::IRManager::Data new_ir = stm.map.getIRData();
    KobukiManager::RunData new_rundata = stm.map.getRunData();

    int i;
    unsigned char signal;
    KobukiManager::State state;

    /* check bumper */
    signal = 0x00;    // check signal
    if( new_sensor.left_bumper  ) signal = 0x01;
    if( new_sensor.center_bumper) signal = 0x01;
    if( new_sensor.right_bumper ) signal = 0x01;
    if( signal ^ last_bumper_signal ){
      if( signal ) stm.sendEvent( BUMPER_PRESSED );
      else stm.sendEvent( BUMPER_RELEASED );
    }
    last_bumper_signal = signal;  // update signal

    /* check IR */
    signal = 0x00;    // check signal
    for( i=0; i<new_ir.size(); i++ ){
      if( new_ir[i].far_left  ) signal = 0x01;
      if( new_ir[i].far_center) signal = 0x01;
      if( new_ir[i].far_right ) signal = 0x01;
      if( new_ir[i].near_left  ) signal = 0x01;
      if( new_ir[i].near_center) signal = 0x01;
      if( new_ir[i].near_right ) signal = 0x01;
    }
    if( signal ^ last_ir_signal ){
      if( signal ) stm.sendEvent( IR_FOUND );
      else stm.sendEvent( IR_MISSED );
    }
    last_ir_signal = signal;  // update signal
  
    /* check RunState */
    state = new_rundata.state;
    if( state != last_run_state ){
      if( state == KobukiManager::STOP )
        if( last_run_state == KobukiManager::RUN ){
          stm.sendEvent( RUN_REACHED );
        }
        else if( last_run_state == KobukiManager::TURN ){
          stm.sendEvent( TURN_REACHED );
        }
    }
    last_run_state = state;   // update state

  }

/*  void sendEvent( int mes ){
    std::cout << "sendEvent [" << mes << "]" << std::endl;
   }
*/
private:

  unsigned char last_bumper_signal;
  unsigned char last_ir_signal;
  KobukiManager::State last_run_state;

};

