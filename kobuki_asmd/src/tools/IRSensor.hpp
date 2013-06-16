/*
 * This text is written by Ryosuke Ishida(ASMD).
/**
 * @file /kobuki_driver/include/kobuki_driver/modules/IRSensor.hpp
 *
 * @brief DockIR Utilities.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#pragma once

#ifndef KOBUKI_DOCK_DRIVE_HPP_
#define KOBUKI_DOCK_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

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
    CENTER=0,
    RIGHT =0,
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
  

  IRManager(){ data.resize(3); };
  //~IRManager();

  bool init(){ return true; }

void update( const DockIR::Data data ){ update( data.docking ); }

void update( const std::vector<unsigned char> &signal ){
  // keep sensor value for past 20 times.
  past_signals.push_back(signal);
  unsigned int window = 20;
  while (past_signals.size() > window)
    past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - window );

  // filtering
  std::vector<unsigned char> signal_filt(signal.size(), 0);
  for ( unsigned int i=0; i<past_signals.size(); i++) {
    if (signal_filt.size() != past_signals[i].size()) continue;
    for (unsigned int j=0; j<signal_filt.size(); j++)
      signal_filt[j] |= past_signals[i][j];
  }

  // set signals
  for (unsigned int i=0; i<3; i++) {
    if (signal_filt[2-i]&FAR_LEFT   ) data[i].far_left   =1; else data[i].far_left   =0;
    if (signal_filt[2-i]&FAR_CENTER ) data[i].far_center =1; else data[i].far_center =0;
    if (signal_filt[2-i]&FAR_RIGHT  ) data[i].far_right  =1; else data[i].far_right  =0;
    if (signal_filt[2-i]&NEAR_LEFT  ) data[i].near_left  =1; else data[i].near_left   =0;
    if (signal_filt[2-i]&NEAR_CENTER) data[i].near_center=1; else data[i].near_center =0;
    if (signal_filt[2-i]&NEAR_RIGHT ) data[i].near_right =1; else data[i].near_right  =0;
  }
}

void showMe() {
  std::string far_signal  = "[F: "; //far field
  std::string near_signal = "[N: "; //near field    
  for (unsigned int i=0; i<3; i++) {
    if (data[i].far_left   ) far_signal  += "L"; else far_signal  += "-";
    if (data[i].far_center ) far_signal  += "C"; else far_signal  += "-";
    if (data[i].far_right  ) far_signal  += "R"; else far_signal  += "-";
    if (data[i].near_left  ) near_signal += "L"; else near_signal += "-";
    if (data[i].near_center) near_signal += "C"; else near_signal += "-";
    if (data[i].near_right ) near_signal += "R"; else near_signal += "-";
    far_signal  += " ";
    near_signal += " ";
  }
  far_signal  += "]";
  near_signal += "]";
  std::cout << far_signal << near_signal << std::endl;
}

  std::vector<IRSignal> data;
private:
  std::vector<std::vector<unsigned char> > past_signals;

};

} // namespace kobuki

#endif /* KOBUKI_DOCK_DRIVE_HPP_ */
