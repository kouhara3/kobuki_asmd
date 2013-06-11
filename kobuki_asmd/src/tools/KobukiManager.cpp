/**
 * @file /kobuki_driver/src/test/KobukiManager.cpp
 *
 * @manager of kobuki.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <iostream>
#include <math.h>
#include <kobuki_driver/kobuki.hpp>

/*****************************************************************************
** Classes
*****************************************************************************/
#define ENCODER_MAX 65535
#define ENCODER_PER_CENTIMETER 117

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
    
    this->coord_now = Coordinate(0,0);
    this->left_encoder_old = kobuki.getCoreSensorData().left_encoder;
    this->right_encoder_old = kobuki.getCoreSensorData().right_encoder;
    this->heading = kobuki.getHeading();
  }

  void spinOnce() {

  }

  /*
   * Called whenever the kobuki receives a data packet. Up to you from here to process it.
   *
   * Note that special processing is done for the various events which discretely change
   * state (bumpers, cliffs etc) and updates for these are informed via the xxxEvent
   * signals provided by the kobuki driver.
   */
  void processStreamData() {
    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;
  }

  //calculate advanced distance of the two wheel 
  void calcDistaceAdvanced(){

    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    int left_encoder_dist = data.left_encoder - this.left_encoder_old;
    int left_encoder_dist = data.right_encoder - this.tight_encoder_old;
    this->heading += kobuki.getHeading();

    //get encoder advanced of left wheel
    if( left_encoder_dist < 50000 ) {
        left_encoder_dist = left_encoder_dist + ENCODER_MAX;
    } else if (left_encoder_dist > 50000) { 
        left_encoder_dist = left_encoder_dist - ENCODER_MAX;
    } else {
        left_encoder_dist = left_encoder_dist;
    }
    coord_left += convertDistToCoordinate(left_encoder_dist, this->heading);

    //get encoder advaced of right wheel
    if( right_encoder_dist < 50000 ) {
        right_encoder_dist = right_dist + ENCODER_MAX;
    } else if (right_encoder_dist > 50000) { 
        right_encoder_dist = right_encoder_dist - ENCODER_MAX;
    } else {
        right_encoder_dist = right_encoder_dist;
    }
    coord_right += convertDistToCoordinate(left_encoder_dist, this->heading);
    
    return;
  }

private:
  Coordinate coord_left;
  Coordinate coord_right;
  int left_encoder_old;
  int right_encoder_old;
  ecl::Angle<double> heading;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;

  //change encoder to centimeter
  int changeEncoderToCentimeter(int encoder){
    int centim;
    centim = (int) (encoder/(folat)ENCODER_PER_CENTIMETER);
    cout<< "Change encoder [" << encoder << "] to centimeter [" << centim << "]" << endl;
    return centim;
  }
  //convert distance and anle to coordinate
  Coordinate convertDistToCoordinate(int dist, ecl::Angle<double> angle) {
    int dist_X = cos(this->heading) * dist;
    int dist_Y = sin(this->heading) * dist;
    Coordinate coord(dist_X, dist_Y);
    return coord;
  }
};

/*****************************************************************************
** Test Main
*****************************************************************************/

int main() {
  KobukiManager kobuki_manager;
  //kobuki_manager.spin();
  return 0;
}
