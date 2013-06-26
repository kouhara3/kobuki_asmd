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

/*
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
*/

class IRManager {
public:
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
  enum IRNUM {
    LEFT  =0,
    CENTER=1,
    RIGHT =2,
  };

  struct IRSignal{
    unsigned char far_left;
    unsigned char far_center;
    unsigned char far_right;
    unsigned char near_left;
    unsigned char near_center;
    unsigned char near_right;

    IRSignal(){
      far_left=0; far_center=0; far_right=0;
      near_left=0; near_center=0; near_right=0;
    }
  };
  
  typedef std::vector<IRSignal> Data;

  IRManager(){ data.resize(3); };
  //~IRManager();

  bool init(){ return true; }

  void update( const DockIR::Data data ){ update( data.docking ); }

  void update( const std::vector<unsigned char> &signal ){
    // keep sensor value for past 20 times.
    past_signals.push_back(signal);
    unsigned int window = 20;
    while ( past_signals.size() > window )
    past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - window );

    // filtering
    std::vector<unsigned char> signal_filt(signal.size(), 0);
    for( unsigned int i=0; i<past_signals.size(); i++ ) {
      if( signal_filt.size() != past_signals[i].size() ) continue;
      for( unsigned int j=0; j<signal_filt.size(); j++ ) signal_filt[j] |= past_signals[i][j];
    }

    // set signals
    for( unsigned int i=0; i<3; i++ ) {
      if( signal_filt[2-i]&FAR_LEFT   ) data[i].far_left   =1; else data[i].far_left   =0;
      if( signal_filt[2-i]&FAR_CENTER ) data[i].far_center =1; else data[i].far_center =0;
      if( signal_filt[2-i]&FAR_RIGHT  ) data[i].far_right  =1; else data[i].far_right  =0;
      if( signal_filt[2-i]&NEAR_LEFT  ) data[i].near_left  =1; else data[i].near_left   =0;
      if( signal_filt[2-i]&NEAR_CENTER) data[i].near_center=1; else data[i].near_center =0;
      if( signal_filt[2-i]&NEAR_RIGHT ) data[i].near_right =1; else data[i].near_right  =0;
    }
  }

  void showMe() {
    std::string far_signal  = "[F: "; // far field
    std::string near_signal = "[N: "; // near field
    for( unsigned int i=0; i<3; i++ ) {
      if( data[i].far_left   ) far_signal  += "L"; else far_signal  += "-";
      if( data[i].far_center ) far_signal  += "C"; else far_signal  += "-";
      if( data[i].far_right  ) far_signal  += "R"; else far_signal  += "-";
      if( data[i].near_left  ) near_signal += "L"; else near_signal += "-";
      if( data[i].near_center) near_signal += "C"; else near_signal += "-";
      if( data[i].near_right ) near_signal += "R"; else near_signal += "-";
      far_signal  += " ";
      near_signal += " ";
    }
    far_signal  += "]";
    near_signal += "]";
    std::cout << far_signal << near_signal << std::endl;
  }

  Data getData(){ return( data ); }   // get signal data

  Data data;
private:
  std::vector<std::vector<unsigned char> > past_signals;

};

} // end of namespace kobuki
