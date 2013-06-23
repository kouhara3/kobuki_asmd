#pragma once

namespace kobuki{

class SensorManager{
public:
  class Data{
  public:
    char left_bumper;
    char center_bumper;
    char right_bumper;
    Data(){ left_bumper=0; center_bumper=0; right_bumper=0;}
  };

  void update( kobuki::CoreSensors::Data& new_data ){
    // check bumpers
    if ((new_data.bumper ^ last_data.bumper) & CoreSensors::Flags::LeftBumper) {
      if (new_data.bumper & CoreSensors::Flags::LeftBumper) {
        data.left_bumper = 1;
      } else {
        data.left_bumper = 0;;
      }
    }
    if ((new_data.bumper ^ last_data.bumper) & CoreSensors::Flags::CenterBumper) {
      if (new_data.bumper & CoreSensors::Flags::CenterBumper) {
        data.center_bumper = 1;
      } else {
        data.center_bumper = 0;;
      }
    }
    if ((new_data.bumper ^ last_data.bumper) & CoreSensors::Flags::RightBumper) {
      if (new_data.bumper & CoreSensors::Flags::RightBumper) {
      } else {
        data.right_bumper = 0;;
      }
    }
    // update data
    last_data = new_data;
  }

  Data getData(){ return( data ); }   // get row signal

private:
  kobuki::CoreSensors::Data last_data;
public:
  Data data;
};

class SimpleRun{
public:
  enum State{
    STOP,
    RUN,
    TURN,
  }

  class Data{
  public:
    State state = STOP;
    double dx, dth, dx_goal, dth_goal, stri_speed, dirct_speed;
    
    Data(){ dx=0.0; dth=0.0; dx_goal=0.0; dth_goal=0.0; stri_speed=0.0; dirct_speed=0.0; }
  };

  void goStright(double _stri_speed, double distance) {
    stopRun();
    data.stri_speed = _stri_speed;
    data.dirct_speed = 0;
    data.dx_goal = distance;
    kobuki.setBaseControl( data.stri_speed, data.dirct_speed ); 
    data.state = RUN;
    return;
  }

  void changeDirction(double _dirct_speed, double angle) {
    stopRun();
    data.stri_speed = 0;
    data.dirct_speed = _dirct_speed;
    data.dth_goal = angle * ecl::pi /180;
    kobuki.setBaseControl( data.stri_speed, data.dirct_speed ); // dirct_speed: to control the speed of dirction
    dta.state = TURN;
    return;
  }

  void stopRun() {
    data.dx=0.0;
    data.dth=0.0;
    data.dx_goal=0.0;
    data.dth_goal=0.0;
    kobuki.setBaseControl(0.0, 0.0);
    data.state = STOP;
    return;
  }

  void update( Kobuki& kobuki ){
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    pose *= pose_update;
    dx += pose_update.x();
    dth += pose_update.heading();

    if( state == RUN ){
      if (data.dx >= data.dx_goal) {
        stopRun();
      } else if (data.dx <= -data.dx_goal) {
        stopRun();
      }
    }
    else if( state == TURN ){
      if (data.dth >= data.dth_goal) {
        stopRun();
      } else if (data.dth <= -data.dth_goal) {
        stopRun();
      }
    }
    return;
  }

  void changeRunSpeed( double new_spd ){
    if( state != RUN ) return;
    data.stri_speed = new_speed;
    kobuki.setBaseControl( data.stri_speed, data.dirct_speed);
    return;
  }

  void changeTurnSpeed( double new_spd ){
    if( state != TURN ) return;
    data.dirct_speed = new_speed;
    kobuki.setBaseControl( data.stri_speed, data.dirct_speed);
    return;
  }

  Data getData(){ return( data ); }

private:
  Data data;

};

} // end of namespace kobuki

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
    BUMPER_PRESSED,
    BUMPER_RELEASED,
    IR_FOUND,
    IR_MISSED,
    RUN_REACHED,
    TURN_REACHED,
  };

  STMEventManager(){
    last_bumper_signal = 0x00;
    last_ir_signal = 0x00;
    last_run_state = 0;
  }

  void checkEvent( kobuki::SensorManager::Data new_sensor,
    kobuki::IRManager::Data new_ir,
    kobuki::SimpleRun::Data new_rundata ){
    int i;
    unsigned char signal;
    kobuki::SimpleRun::State state;

    /* check bumper */
    signal = 0x00;    // check signal
    if( new_sensor.left_bumper  ) signal = 0x01;
    if( new_sensor.center_bumper) signal = 0x01;
    if( new_sensor.right_bumper ) signal = 0x01;
    if( signal ^ last_bumper_signal ){
      if( signal ) sendEvent( BUMPER_PRESSED );
      else sendEvent( BUMPER_RELEASED );
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
      if( signal ) sendEvent( IR_FOUND );
      else sendEvent( IR_MISSED );
    }
    last_ir_signal = signal;  // update signal
  
    /* check RunState */
    state = new_rundata.state;
    if( state != last_run_state ){
      if( state == kobuki::SimpleRun::State::STOP )
        if( last_run_state == kobuki::SimpleRun::State::RUN ){
          sendEvent( RUN_REACHED );
        }
        else if( last_run_state == kobuki::SimpleRun::State::TURN ){
          sendEvent( TURN_REACHED );
        }
    }
    last_run_state = state;   // update state

  }

  void sendEvent( int mes ){
    std::cout << "sendEvent [" << mes << "]" << std::endl;
   }

private:
  unsigned char last_bumper_signal;
  unsigned char last_ir_signal;
  kobuki::SimpleRun::State last_run_state;

};

