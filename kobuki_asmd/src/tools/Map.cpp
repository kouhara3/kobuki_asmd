/**
 * @file /kobuki_driver/src/test/Coordinate.cpp
 *
 * @Coordinate.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
/*****************************************************************************
** Classes
*****************************************************************************/

class Coordinate {
public:
  Map(){
    this->max;
    this->Y = 0;
  }
  Map(int coord_x, coord_y){
    this->X = coord_x;
    this->Y = coord_y;
  }


private:
  Coordinate left_top;
  Coordinate right_top;
  Coordinate left_bottom;
  Coordinate right_bottom;
};

/*****************************************************************************
** Test Main
*****************************************************************************/

int main() {
    return 0;
}
