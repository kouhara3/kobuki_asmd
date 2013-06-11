/**
 * @file /kobuki_driver/src/test/KobukiManager.cpp
 *
 * @manager of kobuki.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <math.h>

/*****************************************************************************
** Classes
*****************************************************************************/
class Utilities {
public:
  //change encoder to centimeter
  static int changeEncoderToCentimeter(int encoder){
    int centim;
    centim = (int) (encoder/(folat)ENCODER_PER_CENTIMETER);
    cout<< "Change encoder [" << encoder << "] to centimeter [" << centim << "]" << endl;
    return centim;
  }
  //convert distance and anle to coordinate
  static Coordinate convertDistToCoordinate(int dist, ecl::Angle<double> angle) {
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
  return 0;
}
