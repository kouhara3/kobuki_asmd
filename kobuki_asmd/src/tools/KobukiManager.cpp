/**
 * @file /kobuki_driver/src/test/KobukiManager.cpp
 *
 * @manager of kobuki.
 **/
#pragma once
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <iostream>
#include <math.h>
#include <kobuki_driver/kobuki.hpp>

#include "Utilities.cpp"

/*****************************************************************************
** Classes
*****************************************************************************/
#define ENCODER_MAX 65535

class KobukiManager {
public:
  KobukiManager() :
      slot_stream_data(&KobukiManager::processStreamData, *this) // establish the callback
  {
    std::cout<< "KobukiManager start" << std::endl;
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/kobuki";         // the serial port to connect to (windows COM1..)
    std::cout<< "Kobuki parameter ok" << std::endl;
    // configure other parameters here
    kobuki.init(parameters);
    std::cout<< "Kobuki init ok" << std::endl;
    slot_stream_data.connect("/mobile_base/stream_data");
    std::cout<< "Kobuki connect ok" << std::endl;

    this->coord_left.setCoordinate(0, 0);
    this->coord_right.setCoordinate(0, 0);
    this->left_encoder_old = kobuki.getCoreSensorData().left_encoder;
    this->right_encoder_old = kobuki.getCoreSensorData().right_encoder;
    this->heading = kobuki.getHeading();
    this->initialPose.setCoordinate(0,0);
    
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
    std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    return this->pose;
  }

/*
 * Simple Run Methods (Now Printing)
 */
// void goStraight( double run_speed, double distance );
// void changeDirection( double turn_speed, doule angle );
// void stopRun(); 

private:
  Coordinate coord_left;
  Coordinate coord_right;
  int left_encoder_old;
  int right_encoder_old;

  Coordinate old_coord;
  Coordinate now_coord;
  ecl::Angle<double> heading;
  ecl::Slot<> slot_stream_data;
  ecl::Pose2D<double> pose;
  Coordinate initialPose;
  kobuki::Kobuki kobuki;

};

/*****************************************************************************
** Test Main
*****************************************************************************/
/*
int main() {
  try{
    KobukiManager kobuki_manager;
    kobuki_manager.spin();
    std::cout << "[" << kobuki_manager.getPose().x() << ", " << kobuki_manager.getPose().y() << ", " << kobuki_manager.getPose().heading() << "]" << std::endl;
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
*/
