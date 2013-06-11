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
  Coordinate(){
    this->X = 0;
    this->Y = 0;
  }
  Coordinate(int coord_x, coord_y){
    this->X = coord_x;
    this->Y = coord_y;
  }

  void setCoordinate(int coord_x, int coord_y){
    this->X = coord_x;
    this->Y = coord_y;
  }
  int getCoordinateX(){
    return this->X;
  }
  int getCoordinateY(){
    return this->Y;
  }

  /***********************************
  **operation of Cordinate
  ************************************/
  bool operator<(const Coordinate &coord) const{
    return ( this->X < coord.X && this->Y < coord.Y);
  }

  bool operator>(const Coordinate &coord) const{
    return ( (this->X > coord.X) && (this->Y > coord.Y));
  }

  Coordinate & operator=(const Coordinate &coord){
    this->X = coord.X;
    this->Y = coord.Y;
    return *this;
  }
  
  Coordinate operator+(Coordinate &coord1){
    Coordinate coord2;
    coord2.X = this->X + coord1.X;
    coord2.X = this->X + coord1.X;
    return t2;
  } 

  Coordinate &operator+=(const Coordinate &coord){
    this->X += coord.X;
    this->Y += coord.Y;
    return *this;
  }
  
  Coordinate operator-(Coordinate &coord1){
    Coordinate coord2;
    coord2.X = this->X - coord1.X;
    coord2.X = this->X - coord1.X;
    return t2;
  } 

  Coordinate &operator-=(const Coordinate &coord){
    this->X -= coord.X;
    this->Y -= coord.Y;
    return *this;
  }

  bool operator==(const Coordinate &coord)const{
    return ((this->X == coord.X) && (this->Y == coord.Y));
  }  

private:
  int X;
  int Y;
};

/*****************************************************************************
** Test Main
*****************************************************************************/

int main() {
    return 0;
}
