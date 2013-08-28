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

#define stringfy(x) #x
#define setState(x) {state=x;state_str=stringfy(x);}
#define setVel(v,w) {vx=v;wz=w;}
#define setStateVel(x,v,w) {setState(x);setVel(v,w);}

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
    rotated += pose_update.heading()/(2.0*M_PI);

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

  void docking( void ){
    enum Station {
      NEAR_LEFT=1,
      NEAR_CENTER=2,
      NEAR_RIGHT=4,
      FAR_CENTER=8,
      FAR_LEFT=16,
      FAR_RIGHT=32,
      NEAR = 7,
      FAR = 56,
    };
    enum State {
      IDLE,
      LOST,
      UNKNOWN,
      INSIDE_FIELD,
      AWAY,
      SCAN,
      SPIN,
      SPIRAL,
      FIND_STREAM,
      GET_STREAM,
      ALIGNED,
      ALIGNED_FAR,
      ALIGNED_NEAR,
      BUMPED,
      BUMPED_DOCK,
      RUN,
      STOP,
      DOCKED_IN,
      DONE,
    };
  
    State state = IDLE;
    double vx=0.0, wz=0.0;
  
    bool is_enabled=false, can_run=false;
    std::string state_str, debug_str;
    int bump_remainder=0;
    int dock_stabilizer=0;
    int dock_detector=0;
    std::vector<std::vector<unsigned char> > past_signals;
    bool rotate_reverse=false;
  
  while( state!=DONE ){
      // update sensor data
      kobuki::SensorManager::Data sensor = getSensorData();
      kobuki::IRManager::Data ir = getIRData();
    
      unsigned char bumper=0;
      if( sensor.left_bumper == 1 ) bumper += 0x04;
      if( sensor.center_bumper == 1 ) bumper += 0x02;
      if( sensor.right_bumper == 1 ) bumper += 0x01;
    
      unsigned char charger=0;
      if( sensor.charger == 1 ) charger += 0x06;
    
      std::vector<unsigned char> signal(3,0);
      for( int i=0; i<ir.size(); i++ ){
        if( ir[i].far_left  ) signal[2-i] = FAR_LEFT;
        if( ir[i].far_center) signal[2-i] = FAR_CENTER;
        if( ir[i].far_right ) signal[2-i] = FAR_RIGHT;
        if( ir[i].near_left  ) signal[2-i] = NEAR_LEFT;
        if( ir[i].near_center) signal[2-i] = NEAR_CENTER;
        if( ir[i].near_right ) signal[2-i] = NEAR_RIGHT;
      }
    
      past_signals.push_back(signal);
      unsigned int window = 20;
      while (past_signals.size() > window)
        past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - window );
    
      std::vector<unsigned char> signal_filt(signal.size(), 0);
      for ( unsigned int i=0; i<past_signals.size(); i++) {
        if (signal_filt.size() != past_signals[i].size()) continue;
        for (unsigned int j=0; j<signal_filt.size(); j++)
          signal_filt[j] |= past_signals[i][j];
      }
    
      // running
      debug_str = "";
      do {  // a kind of hack
        if ( state==DONE ) setState(IDLE); // when this function is called after final state 'DONE'.
        if ( state==DOCKED_IN ) {
          if ( dock_stabilizer++ > 20 ) {
            is_enabled = false;
            can_run = false;
            setStateVel(DONE, 0.0, 0.0); break;
          }
          setStateVel(DOCKED_IN, 0.0, 0.0); break;
        }
        if ( bump_remainder > 0 ) {
          bump_remainder--;
          if ( charger ) { setStateVel(DOCKED_IN, 0.0, 0.0); break; } // when bumper signal is received early than charger(dock) signal.
          else {           setStateVel(BUMPED_DOCK, -0.01, 0.0); break; }
        } else if (state == BUMPED) {
          setState(IDLE); //should I remember and recall previous state?
          debug_str="how dare!!";
        }
        if ( bumper || charger ) {
          if( bumper && charger ) {
            bump_remainder = 0;
            setStateVel(BUMPED_DOCK, -0.01, 0.0); break;
          }
          if ( bumper ) {
            bump_remainder = 50;
            setStateVel(BUMPED, -0.05, 0.0); break;
          }
          if ( charger ) { // already docked in
            dock_stabilizer = 0;
            setStateVel(DOCKED_IN, 0.0, 0.0); break;
          }
        } else {
          if ( state==IDLE ) {
            dock_detector = 0;
            setStateVel(SCAN, 0.00, 0.66); break;
          }
          if ( state==SCAN ) {
            std::ostringstream oss;
            oss << "rotated: " << std::fixed << std::setprecision(2) << std::setw(4) << rotated;
            debug_str = oss.str();
            if( std::abs(rotated) > 1.6 ) {
              setStateVel(FIND_STREAM, 0.0, 0.0); break;
            }
            if (  signal_filt[1]&(FAR_LEFT  + NEAR_LEFT )) dock_detector--;
            if (  signal_filt[1]&(FAR_RIGHT + NEAR_RIGHT)) dock_detector++;
            if ( (signal_filt[1]&FAR_CENTER) || (signal_filt[1]&NEAR_CENTER) ) {
              setStateVel(ALIGNED, 0.05, 0.00); break;
            } else if ( signal_filt[1] ) {
              if( rotate_reverse ){
                setStateVel(SCAN, 0.00, -0.20);
                break;
              } else {
                setStateVel(SCAN, 0.00, 0.20);
                break;
              }
            } else {
              if( rotate_reverse ){
                setStateVel(SCAN, 0.00, -0.66);
                break;
              } else { 
                setStateVel(SCAN, 0.00, 0.66);
                break;
              }
            }
    
          } else if (state==ALIGNED || state==ALIGNED_FAR || state==ALIGNED_NEAR) {
            if ( signal_filt[1] ) {
              if ( signal_filt[1]&NEAR )
              {
                if ( ((signal_filt[1]&NEAR) == NEAR_CENTER) || ((signal_filt[1]&NEAR) == NEAR) ) { setStateVel(ALIGNED_NEAR, 0.05,  0.0); debug_str = "AlignedNearCenter"; break; }
                if (   signal_filt[1]&NEAR_LEFT  ) {                                               setStateVel(ALIGNED_NEAR, 0.05,  0.1); debug_str = "AlignedNearLeft"  ; break; }
                if (   signal_filt[1]&NEAR_RIGHT ) {                                               setStateVel(ALIGNED_NEAR, 0.05, -0.1); debug_str = "AlignedNearRight" ; break; }
              }
              if ( signal_filt[1]&FAR )
              {
                if ( ((signal_filt[1]&FAR) == FAR_CENTER) || ((signal_filt[1]&FAR) == FAR) ) { setStateVel(ALIGNED_FAR, 0.1,  0.0); debug_str = "AlignedFarCenter"; break; }
                if (   signal_filt[1]&FAR_LEFT  ) {                                            setStateVel(ALIGNED_FAR, 0.1,  0.3); debug_str = "AlignedFarLeft"  ; break; }
                if (   signal_filt[1]&FAR_RIGHT ) {                                            setStateVel(ALIGNED_FAR, 0.1, -0.3); debug_str = "AlignedFarRight" ; break; }
              }
              dock_detector = 0;
              rotated = 0.0;
              setStateVel(SCAN, 0.00, 0.66); break;
            } else {
              debug_str = "lost signals";
              setStateVel(LOST, 0.00, 0.00); break;
            }
          } else if (state==FIND_STREAM) {
            if (dock_detector > 0 ) { // robot is placed in right side of docking station
              //turn  right , negative direction til get right signal from left sensor
              if (signal_filt[2]&(FAR_RIGHT+NEAR_RIGHT)) {
                setStateVel(GET_STREAM, 0.05, 0.0); break;
              } else {
                setStateVel(FIND_STREAM, 0.0, -0.33); break;
              }
            } else if (dock_detector < 0 ) { // robot is placed in left side of docking station
              //turn left, positive direction till get left signal from right sensor
              if (signal_filt[0]&(FAR_LEFT+NEAR_LEFT)) {
                setStateVel(GET_STREAM, 0.05, 0.0); break;
              } else {
                setStateVel(FIND_STREAM, 0.0, 0.33); break;
              }
            }
          } else if (state==GET_STREAM) {
            if (dock_detector > 0) { //robot is placed in right side of docking station
              if (signal_filt[2]&(FAR_LEFT+NEAR_LEFT)) {
                dock_detector = 0;
                rotated = 0.0;
                rotate_reverse = false;
                setStateVel(SCAN, 0.0, 0.20); break;
              } else {
                setStateVel(GET_STREAM, 0.05, 0.0); break;
              }
            } else if (dock_detector < 0) { // robot is placed in left side of docking station
              if (signal_filt[0]&(FAR_RIGHT+NEAR_RIGHT)) {
                dock_detector = 0;
                rotated = 0.0;
                rotate_reverse = true;
                setStateVel(SCAN, 0.0, -0.20); break;
              } else {
                setStateVel(GET_STREAM, 0.05, 0.0); break;
              }
            }
          } else {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(SCAN, 0.00, 0.66); break;
          }
        }
        setStateVel(UNKNOWN, 0.00, 0.00); break;
      } while(0);
    
      rundata.stri_speed = vx;
      rundata.dirct_speed = wz;
      kobuki.setBaseControl( rundata.stri_speed, rundata.dirct_speed);
    }
  stopRun();

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
  double rotated;

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
