/**
 * @file /kobuki_driver/src/test/KobukiManager.cpp
 *
 * @manager of kobuki.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#pragma once

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <iostream>
#include <math.h>
#include <kobuki_driver/kobuki.hpp>

#include "Utilities.cpp"
#include "Sensors.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/
#define ENCODER_MAX 65535


class KobukiManager {
public:
  enum State{
    STOP,
    RUN,
    TURN,
  };

  class RunData{
  public:
    State state;
    double dx, dth, dx_goal, dth_goal, stri_speed, dirct_speed;
    
    RunData(){ state = STOP; dx=0.0; dth=0.0; dx_goal=0.0; dth_goal=0.0; stri_speed=0.0; dirct_speed=0.0; }
  };

  KobukiManager() :
      slot_stream_data(&KobukiManager::processStreamData, *this) // establish the callback
  {
    std::cout<< "KobukiManager start" << std::endl;
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/kobuki";         // the serial port to connect to (windows COM1..)
    parameters.enable_acceleration_limiter = false;
    std::cout<< "Kobuki parameter ok" << std::endl;
    // configure other parameters here
    kobuki.init(parameters);
    kobuki.enable();
    std::cout<< "Kobuki init ok" << std::endl;
    slot_stream_data.connect("/mobile_base/stream_data");
    std::cout<< "Kobuki connect ok" << std::endl;

    this->coord_left.setCoordinate(0, 0);
    this->coord_right.setCoordinate(0, 0);
    this->left_encoder_old = kobuki.getCoreSensorData().left_encoder;
    this->right_encoder_old = kobuki.getCoreSensorData().right_encoder;
    this->heading = kobuki.getHeading();
    
  }

  ~KobukiManager() {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void spin() {
    std::cout<< "KobukiManager OK!" << std::endl;
  }

  /*
   * Called whenever the kobuki receives a data packet. Up to you from here to process it.
   *
   * Note that special processing is done for the various events which discretely change
   * state (bumpers, cliffs etc) and updates for these are informed via the xxxEvent
   * signals provided by the kobuki driver.
   */
  void processStreamData() {
    
    //get now pose of kobuki
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    this->pose *= pose_update;

    kobuki::CoreSensors::Data core_data = kobuki.getCoreSensorData();
    kobuki::DockIR::Data ir_data = kobuki.getDockIRData();
    core_sensor.update( core_data );
    ir_sensor.update( ir_data );

    runUpdate( pose_update );

    //kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    //std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;
    
  }
  
  /*
   * Calculate advanced distance of the two wheel.
   */ 
  void calcDistaceAdvanced(){

    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    int left_encoder_dist = data.left_encoder - this->left_encoder_old;
    int right_encoder_dist = data.right_encoder - this->right_encoder_old;
    this->heading += kobuki.getHeading();

    //get encoder advanced of left wheel
    if( left_encoder_dist < 50000 ) {
        left_encoder_dist = left_encoder_dist + ENCODER_MAX;
    } else if (left_encoder_dist > 50000) { 
        left_encoder_dist = left_encoder_dist - ENCODER_MAX;
    } else {
        left_encoder_dist = left_encoder_dist;
    }
    coord_left += Utilities::convertDistToCoordinate(left_encoder_dist, this->heading);

    //get encoder advaced of right wheel
    if( right_encoder_dist < 50000 ) {
        right_encoder_dist = right_encoder_dist + ENCODER_MAX;
    } else if (right_encoder_dist > 50000) { 
        right_encoder_dist = right_encoder_dist - ENCODER_MAX;
    } else {
        right_encoder_dist = right_encoder_dist;
    }
    coord_right += Utilities::convertDistToCoordinate(right_encoder_dist, this->heading);
    
    return;
  }

  /*
   * Get coordinate of left wheel.
   */ 
  Coordinate getCoordLeft() {
    return this->coord_left;
  }
  
  /*
   * Get coordinate of left wheel.
   */ 
  Coordinate getgetCoordRight() {
    return this->coord_left;
  }

  /*
   * Get now pose of kobuki.
   */ 
  ecl::Pose2D<double> getPose() {
    std::cout << "Current Position: [" << pose.x() << ", " << pose.y() << ", " << pose.heading().degrees() << "]" << std::endl;
    return this->pose;
  }

/*
 * Simple Run Methods (Now Printing)
 */
// void goStraight( double run_speed, double distance );
// void changeDirection( double turn_speed, doule angle );
// void stopRun(); 

  void goStraight(double _stri_speed, double distance) {
    stopRun();
    rundata.stri_speed = _stri_speed;
    rundata.dirct_speed = 0;
    rundata.dx_goal = distance;
    kobuki.setBaseControl( rundata.stri_speed, rundata.dirct_speed ); 
    rundata.state = RUN;
    return;
  }

  void changeDirection(double _dirct_speed, double angle) {
    stopRun();
    rundata.stri_speed = 0;
    if( angle >= 0 ){
      rundata.dirct_speed = _dirct_speed;
      rundata.dth_goal = angle * ecl::pi /180;
    }
    else {
      rundata.dirct_speed = -_dirct_speed;
      rundata.dth_goal = -angle * ecl::pi /180;  
    }
    kobuki.setBaseControl( rundata.stri_speed, rundata.dirct_speed ); // dirct_speed: to control the speed of dirction
    rundata.state = TURN;
    return;
  }

  void stopRun() {
    rundata.dx=0.0;
    rundata.dth=0.0;
    rundata.dx_goal=0.0;
    rundata.dth_goal=0.0;
    kobuki.setBaseControl(0.0, 0.0);
    rundata.state = STOP;
    return;
  }

  void runUpdate( ecl::Pose2D<double> pose_update ){
    rundata.dx += pose_update.x();
    rundata.dth += pose_update.heading();
    //std::cout << rundata.state << "," << rundata.dx << std::endl;
    //std::cout << rundata.stri_speed << "," << rundata.dirct_speed << std::endl; 

    if( rundata.state == RUN ){
      if (rundata.dx >= rundata.dx_goal) {
        //std::cout << "Target Distance: " << rundata.dx_goal << std::endl;
        //std::cout << "Run Distance: " << rundata.dx << std::endl;
        stopRun();
      } else if (rundata.dx <= -rundata.dx_goal) {
        stopRun();
      }
    }
    else if( rundata.state == TURN ){
      if (rundata.dth >= rundata.dth_goal) {
        //std::cout << "Target Angle: " << rundata.dth_goal * 180 / ecl::pi << std::endl;
        //std::cout << "Turned Angle: " << rundata.dth * 180 / ecl::pi << std::endl;
        stopRun();

      } else if (rundata.dth <= -rundata.dth_goal) {
        //std::cout << "Target Angle: " << rundata.dth_goal * 180 / ecl::pi << std::endl;
        //std::cout << "Turned Angle: " << rundata.dth * 180 / ecl::pi << std::endl;
        stopRun();
      }
    }
    return;
  }

  void changeRunSpeed( double new_speed ){
    if( rundata.state != RUN ) return;
    rundata.stri_speed = new_speed;
    kobuki.setBaseControl( rundata.stri_speed, rundata.dirct_speed);
    return;
  }

  void changeTurnSpeed( double new_speed ){
    if( rundata.state != TURN ) return;
    rundata.dirct_speed = new_speed;
    kobuki.setBaseControl( rundata.stri_speed, rundata.dirct_speed);
    return;
  }

  RunData getRunData(){ return( rundata ); }
  kobuki::IRManager::Data getIRData(){ return( ir_sensor.getData() ); }
  kobuki::SensorManager::Data getSensorData(){ return( core_sensor.getData() ); }

private:
  RunData rundata;
  Coordinate coord_left;
  Coordinate coord_right;
  int left_encoder_old;
  int right_encoder_old;
  ecl::Angle<double> heading;

  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
  ecl::Pose2D<double> pose;
  kobuki::IRManager ir_sensor;
  kobuki::SensorManager core_sensor;

};

/*****************************************************************************
** Test Main
*****************************************************************************/
/*
int main() {
  try{
    KobukiManager kobuki_manager;
    kobuki_manager.spin();
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
*/
