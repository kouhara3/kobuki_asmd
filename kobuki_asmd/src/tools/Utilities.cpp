/**
 * @file /kobuki_driver/src/test/KobukiManager.cpp
 *
 * @usually used utilities.
 **/
#pragma ocne
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/geometry/pose2d.hpp>
#include <iostream>
#include <math.h>
#include "Coordinate.cpp"

/*****************************************************************************
** Definition
*****************************************************************************/

#define ENCODER_PER_CENTIMETER 117

/*****************************************************************************
** Classes
*****************************************************************************/
class Utilities {
public:
  //change encoder to centimeter
  static int changeEncoderToCentimeter(int encoder){
    int centim;
    centim = (int) (encoder/(float)ENCODER_PER_CENTIMETER);
    std::cout<< "Change encoder [" << encoder << "] to centimeter [" << centim << "]" << std::endl;
    return centim;
  }

  //convert distance and angle to coordinate
  static Coordinate convertDistToCoordinate(int dist, ecl::Angle<double> angle) {
    int dist_x = cos(angle) * dist;
    int dist_y = sin(angle) * dist;
    std::cout<< "Convert ["<< dist << ", "<< angle <<"] to" << " [" << dist_x << ", "<< dist_y <<"]" << std::endl;
    Coordinate coord(dist_x, dist_y);
    return coord;
  }
};

/*****************************************************************************
** Test Main
*****************************************************************************/
/*
int main() { 
  Utilities::changeEncoderToCentimeter(11700);
  return 0;
}
*/
