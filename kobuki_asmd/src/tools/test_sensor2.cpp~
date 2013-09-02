/**
 * @file /kobuki_driver/src/test/sigslots.cpp
 *
 * @brief Example/test program for kobuki sigslots.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <iostream>
#include <kobuki_driver/kobuki.hpp>
#include "IRSensor.hpp"
#include "test.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager() :
      slot_stream_data(&KobukiManager::processStreamData, *this) // establish the callback
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/kobuki";         // the serial port to connect to (windows COM1..)
    // configure other parameters here
    kobuki.init(parameters);
    slot_stream_data.connect("/mobile_base/stream_data");
  }

  void spin() {
    ecl::Sleep sleep(1);
    sleep();
  }

  /*
   * Called whenever the kobuki receives a data packet. Up to you from here to process it.
   *
   * Note that special processing is done for the various events which discretely change
   * state (bumpers, cliffs etc) and updates for these are informed via the xxxEvent
   * signals provided by the kobuki driver.
   */
//  void processStreamData() {
//    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
//    std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;
//  }
  void processStreamData() {
    kobuki::CoreSensors::Data core_data = kobuki.getCoreSensorData();
    kobuki::DockIR::Data ir_data = kobuki.getDockIRData();
    core_sensor.update( core_data );
    ir_sensor.update( ir_data );
  }

  kobuki::IRManager::Data getIRData(){ return( ir_sensor.getData() ); }
  kobuki::SensorManager::Data getSensorData(){ return( core_sensor.getData() ); }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
  kobuki::IRManager ir_sensor;
  kobuki::SensorManager core_sensor;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main() {
  KobukiManager kobuki_manager;
  STMEventManager event_manager;
  ecl::Sleep sleep(0.1);
  while( true ){
    //kobuki_manager.spin();
    sleep();
    event_manager.checkEvent( kobuki_manager.getSensorData() , kobuki_manager.getIRData() );
  }
  return 0;
}
