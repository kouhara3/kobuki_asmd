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


private:
  kobuki::CoreSensors::Data last_data;
public:
  Data data;
};

}; // end of namespace kobuki

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
  };

  unsigned char last_bumper_signal;
  unsigned char last_ir_signal;

  STMEventManager(){
    last_bumper_signal = 0x00;
    last_ir_signal = 0x00;
  }

  void checkEvent( kobuki::kobukiManager& kobuki_manager ){
    int i;
    unsigned char signal;
    kobuki::SensorManager::Data& new_sensor = kobuki_manager.

    signal = 0x00;    // check signal
    if( new_sensor.left_bumper  ) signal = 0x01;
    if( new_sensor.center_bumper) signal = 0x01;
    if( new_sensor.right_bumper ) signal = 0x01;
    if( signal ^ last_bumper_signal ){
      if( signal ) sendEvent( BUMPER_PRESSED );
      else sendEvent( BUMPER_RELEASED );
    }
    last_bumper_signal = signal;  // update signal

    signal = 0x00;    // check signal
    for( i=0; i<3; i++ ){
      if( new_ir.data[i].far_left  ) signal = 0x01;
      if( new_ir.data[i].far_center) signal = 0x01;
      if( new_ir.data[i].far_right ) signal = 0x01;
      if( new_ir.data[i].near_left  ) signal = 0x01;
      if( new_ir.data[i].near_center) signal = 0x01;
      if( new_ir.data[i].near_right ) signal = 0x01;
    }
    if( signal ^ last_ir_signal ){
      if( signal ) sendEvent( IR_FOUND );
      else sendEvent( IR_MISSED );
    }
    last_ir_signal = signal;  // update signal
  }

  void sendEvent( int mes ){
    std::cout << "sendEvent [" << mes << "]" << std::endl;
   }
};

