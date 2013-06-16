/**                                                                           !
  * @file /kobuki_driver/src/test/simple_loop.cpp
  *
  * @brief Example/test program with simple loop.
  *
  * It provides simple example of how interact with kobuki by using c++ without ROS.
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <csignal>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include "kobuki_driver/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager() :
    dx(0.0), dth(0.0),
    slot_stream_data(&KobukiManager::processStreamData, *this)
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki";
    parameters.device_port = "/dev/kobuki";
    parameters.enable_acceleration_limiter = false;
    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/kobuki/stream_data");
    printPose();
state = 0;
  }

  ~KobukiManager() {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void processStreamData() {
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    pose *= pose_update;
    dx += pose_update.x();
    dth += pose_update.heading();
    //std::cout << dx << ", " << dth << std::endl;
    //std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
    //std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    processMotion();
  }

 void printPose(){
    //ecl::Pose2D<double> pose_update;
    //ecl::linear_algebra::Vector3d pose_update_rates;
    //kobuki.updateOdometry(pose_update, pose_update_rates);
    //pose *= pose_update;
    //dx += pose_update.x();
    //dth += pose_update.heading();
    //std::cout << dx << ", " << dth << std::endl;
    //std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
    std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
}

  // Generate straight motion ( U-turn )
/*  void processMotion() {
    if (dx >= 2.0 && dth >= ecl::pi) { dx=0.0; dth=0.0; kobuki.setBaseControl(0.0, 0.0); return; }
    else if (dx >= 2.0) { kobuki.setBaseControl(0.0, 3.3); return; }
    else { kobuki.setBaseControl(0.3, 0.0); return; }
  }
*/
  // ( I-turn )
   void processMotion() {
if ( state == 2 ) return;
    if ( dx >= 2.0 ) { printPose(); dx=0.0; dth=0.0; state=1; kobuki.setBaseControl(0.0, 0.0); return; }
    else if ( dx <= -2.0 ) { printPose(); dx=0.0; dth=0.0; state=2; kobuki.setBaseControl(0.0, 0.0); return; }
else if ( (state == 1) && (dx >= -0.2) ) { kobuki.setBaseControl(-0.1, 0.0); return; }
else if ( dx <= -1.8 ) { kobuki.setBaseControl(-0.1, 0.0); return; }
    else if ( state == 1 ) { kobuki.setBaseControl(-0.3, 0.0); return; }
else if ( (state == 0) && (dx <= 0.2) ) { kobuki.setBaseControl(0.1, 0.0); return; }
else if ( dx >= 1.8 ) { kobuki.setBaseControl(0.1, 0.0); return; }
    else { kobuki.setBaseControl(0.3, 0.0); return; }
  }

  ecl::Pose2D<double> getPose() {
    return pose;
  }

private:
int state;
  double dx, dth;
  ecl::Pose2D<double> pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  std::cout << "Demo : Example of simple control loop." << std::endl;
  KobukiManager kobuki_manager;

  ecl::Sleep sleep(1);
  ecl::Pose2D<double> pose;
  try {
    while (!shutdown_req){
      sleep();
      pose = kobuki_manager.getPose();
      //std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
