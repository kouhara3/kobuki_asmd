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
    dx(0.0), dth(0.0), stri_speed (0.0), dirct_speed (0.0),
    slot_stream_data(&KobukiManager::processStreamData, *this)
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki";
    parameters.device_port = "/dev/kobuki";
    parameters.enable_acceleration_limiter = false;
    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/kobuki/stream_data");
  }

  ~KobukiManager() {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void goStright(double _stri_speed, double distance) {
    stri_speed = _stri_speed;
    kobuki.setBaseControl(_stri_speed, dirct_speed); 
    if (dx >= distance) {
      stopRun();
    } else if (dx <= -distance) {
      stopRun();
    }
    return;
  }

  void changeDirction(double _dirct_speed, double angle) {
    dirct_speed = _dirct_speed;
    kobuki.setBaseControl(stri_speed, _dirct_speed); // dirct_speed: to control the speed of dirction
      if (dth >= angle ＊ pi / 180) {
        stopRun();
      } else if (dth <= -angle ＊ pi / 180) {
        stopRun();
      } 
      return;
  }

  void stopRun() {
    dx=0.0; 
    dth=0.0; 
    kobuki.setBaseControl(0.0, 0.0); 
    return;
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
  }

  ecl::Pose2D<double> getPose() {
    return pose;
  }

private:
  double dx, dth, stri_speed, dirct_speed;
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
      kobuki_manager.goStright(0.3, 2);
      //kobuki_manager.changeDirction(0.3, 90);
      pose = kobuki_manager.getPose();
      std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
