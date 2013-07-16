/**
 * @file /kobuki_driver/src/test/Map.cpp
 *
 * @Map in coordinate.
 **/
#pragma once
/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <math.h>
#include <vector>
#include <QApplication>
#include <QWidget>
#include <QPixmap>
#include "Coordinate.cpp"
#include "KobukiManager.hpp"
#include "Block.cpp"
/*****************************************************************************
** Define
*****************************************************************************/
#define MINIMUM_SIZE 0.5
/*****************************************************************************
** Classes
*****************************************************************************/

class Map : public QWidget {
public:
  Map(){
    this->max.setCoordinate(0,0);
    this->current_block = NULL;
    //blockListInit();
  }
  Map(double coord_x, double coord_y){
    this->max.setCoordinate(coord_x, coord_y);
    this->current_block = NULL;
    //blockListInit();
  }
  void reset(double coord_x, double coord_y){
    this->max.setCoordinate(coord_x, coord_y);
    this->current_block = NULL;
    blockListInit();
  }

  bool blockListInit(){
    this->block_list.clear();

    int idx_x = this->max.getCoordinateX()/DEFAULT_BLOCK_LENGTH;
    int idx_y = this->max.getCoordinateY()/DEFAULT_BLOCK_LENGTH;
    std::cout << "idx_x = " << idx_x << std::endl; 
    std::cout << "idx_y = " << idx_y << std::endl;
   

    if( idx_x>0 && idx_y>0 ){
      //Block initialization 
      for(int i = 0; i<idx_x; i++) {
      
        std::vector<Block> list;
      
        for(int j = 0; j<idx_y; j++) {
          Coordinate coord;
          coord.setCoordinate(i*DEFAULT_BLOCK_LENGTH, j*DEFAULT_BLOCK_LENGTH);
          Block block(coord, DEFAULT_BLOCK_LENGTH, i, j);
  	  list.push_back(block);
        }

        this->block_list.push_back(list);   
        list.clear(); 
      }
      
      //Edges initialization
      for(int i = 0; i<idx_x; i++) {

        for(int j = 0; j<idx_y; j++) {

	  if(j+1<idx_y) { this->block_list[i][j].edges.up_edge = (Block*) &this->block_list[i][j+1]; }
          else  { this->block_list[i][j].edges.up_edge = NULL; }

	  if(i+1<idx_x) { this->block_list[i][j].edges.right_edge = (Block*) &this->block_list[i+1][j]; }
          else { this->block_list[i][j].edges.right_edge = NULL; }

	  if(j-1>=0) { this->block_list[i][j].edges.down_edge = (Block*) &this->block_list[i][j-1]; }
          else { this->block_list[i][j].edges.down_edge = NULL; }

	  if(i-1<idx_y) { this->block_list[i][j].edges.left_edge = (Block*) &this->block_list[i-1][j]; }
          else  { this->block_list[i][j].edges.left_edge = NULL; }
        }
      }

      this->current_block = (Block*) &block_list[0][0];
      block_list[0][0].setMark(BLANK);
      block_list[0][0].setHasKobuki(true);
      std::cout << "Block list initialezed successfully." << std::endl;
      std::cout << "Size of block_list is [" << this->block_list.size() <<"]" << std::endl;
      return true;
    } else {
      this->current_block = NULL;
      std::cout << "Block list initialeze failed." << std::endl;
      return false;
    }
  }
  
  Block* getNextBlock(){
    
    if(this->block_list.empty()){
      return NULL;
    }
    
    Block* next_block = NULL;
    int now_x = this->current_block->getTagX();
    int now_y = this->current_block->getTagY();
    
    if( now_y-1>=0 && this->block_list[now_x][now_y-1].isUnkonwn() ){
      next_block = &this->block_list[now_x][now_y-1];
    } else if( now_x+1<this->block_list.size() && this->block_list[now_x+1][now_y].isUnkonwn()){
      next_block = &this->block_list[now_x+1][now_y];
    } else if( now_x-1>=0 && this->block_list[now_x-1][now_y].isUnkonwn()){
      next_block = &this->block_list[now_x-1][now_y];
    } else if( now_y+1< this->block_list[now_x].size() && !this->block_list[now_x][now_y+1].isObstacle()){
      next_block = &this->block_list[now_x][now_y+1];
    } else if( now_x-1>=0 ){
      next_block = &this->block_list[now_x-1][now_y];
    } else{
      next_block = NULL;
    }
/*
//////This block can provide a instant value of angle and distance to be advanced///////////

    double dist_x = this->manager.getPose().x() - next_block->getCenterPoint().getCoordinateX();
    double dist_y = this->manager.getPose().x() - next_block->getCenterPoint().getCoordinateY();
    double angle_now = this->manager.getPose().degrees();

    this->angle_to_next = (atan(dist_x/dist_y)/ecl::pi)*180 - angle_now;
    this->distance_to_next = pow( pow(dist_x, 2.0)+pow(dist_y, 2.0), 0.5);
////////////////////////////////////////////////////////////////////////////////////////////
*/
    //std::cout << "Next Position: " << (float)next_block->getCenterPoint().getCoordinateX()
    //    << " , " << (float)next_block->getCenterPoint().getCoordinateY() << std::endl;
    return next_block;
  }

//Get angle will be turned from now to next block
  double getTurnAngle(Block* next_block){
    //Block* next_block = this->getNextBlock();
    if(next_block == NULL){
      return 0;
    }
    
    //double dist_x = this->current_block->centerPoint.getCoordinateX() - next_block->centerPoint.getCoordinateX();
    //double dist_y = this->current_block->centerPoint.getCoordinateX() - next_block->centerPoint.getCoordinateX();
    ecl::Pose2D<double> pose_now = this->manager.getPose();
    Coordinate next_point = next_block->getCenterPoint();
    double dist_x = next_point.getCoordinateX() - pose_now.x();
    double dist_y = next_point.getCoordinateY() - pose_now.y();
    double angle_now = pose_now.heading().degrees();

    double turn_angle = (atan2(dist_y,dist_x)/ecl::pi)*180 - angle_now;

    if((turn_angle < 5.0) && (turn_angle > -5.0)) turn_angle = 0;
    if(turn_angle>180.0) turn_angle = turn_angle - 360.0;
    if(turn_angle<-180.0) turn_angle = turn_angle + 360.0;
    turn_angle = turn_angle;
    //std::cout << "angle: " << angle_now << std::endl;
    //std::cout << "turn_angle: " << turn_angle << std::endl;
    return turn_angle;
  }

//Get distance will be advaced from now to next block
  double getNextDistance(Block* next_block){

    //Block* next_block = this->getNextBlock();
    if(next_block == NULL){
      return 0;
    }
    
    ecl::Pose2D<double> pose_now = this->manager.getPose();
    Coordinate next_point = next_block->getCenterPoint();
    double dist_x = next_point.getCoordinateX() - pose_now.x();
    double dist_y = next_point.getCoordinateY() - pose_now.y();
    
    double dist = pow( pow(dist_x, 2.0)+pow(dist_y, 2.0), 0.5);

    //std::cout << "x,y,distance: " << dist_x << "," << dist_y << "," << dist << std::endl;

    return dist;
  }

  void getMapInfo(){

    std::cout << "This map has a max coordinate of [" << this->max.getCoordinateX() << "," << this->max.getCoordinateY() << "]" << std::endl;
    if(!block_list.empty()){
      std::cout << "This map has a max block x of [" << this->block_list.size() << "]." << std::endl;
      std::cout << "This map has a max block y of [" << this->block_list.begin()->size() << "]." << std::endl;
    } else {
      std::cout << "This map has no block." << std::endl;
    }
  }
  
  Block* getCurrentBlock(){
    return this->current_block;
  }
  void setCurrentBlock(Block* b){
    this->current_block = b;
    //current_block->setHasKobuki(true);
    return;
  }
  void setMarkBlock(Block* b, Mark m){
    b->setMark(m);
    return;
  }
  KobukiManager* getKobukiManager(){
    return &(this->manager);
  }

  
  void checkWall(){

    int j = 0;
    while(this->block_list[0][j].getMark() != OBSTACLE){
        j++;
    }

    setWall(&this->block_list[0][j]);
  }

  void setWall(Block* wall_block){

  	int idx_x = this->block_list.size();
	int idx_y = this->block_list[0].size();
	int tag_x = wall_block->getTagX();
	int tag_y = wall_block->getTagX();

	if(!(tag_x > 0 && tag_y == 0))
	{
		wall_block->setMark(WALL);

		for(int i = tag_x -1; i<= tag_x + 1; i++)
		{
			if(i == -1) i = 0;
			for(int j = tag_y -1; i<= tag_y + 1; j++)
			{
				if(j == -1) j = 0;
				if(this->block_list[i][j].getMark() == OBSTACLE)
				{
					setWall(&this->block_list[i][j]);
				}
			}
		}
	} 
	else 
	{
		wall_block->setMark(WALL);
	}
      return;
  }

  void showMap(){
    int idx_x = this->block_list.size();
    int idx_y = this->block_list[0].size();
    std::cout << "=====Show now map construct !=====" << std::endl << std::endl;
    for(int i = idx_y-1; i>-1; i--){
      for(int j = 0; j<idx_x ; j++){
        switch(block_list[j][i].getMark()){
          case UNKNOWN: std::cout << "? ";	break;

          case BLANK:	std::cout << "O ";	break;

          case OBSTACLE:std::cout << "X ";	break;
        }
      }
      std::cout << std::endl;
    }
    std::cout << "==================================" << std::endl << std::endl;
    return;
  }

  void ShowMap();

private:
  Coordinate max;
  std::vector< std::vector<Block> > block_list;
  Block* current_block;
  double angle_to_next;
  double distance_to_next;
  KobukiManager manager;
  QPixmap pix;

};

/*****************************************************************************
** Test Main
*****************************************************************************/


int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    Map m(2.5,2.5);
    m.blockListInit();
    m.getMapInfo();
    Block* b = m.getNextBlock();
    if(m.getCurrentBlock()!=NULL)std::cout<<"Kobuki is now at block ["<< m.getCurrentBlock()->getTagX() <<","<< m.getCurrentBlock()->getTagY() <<"]."<<std::endl;
    else std::cout<<"Null block position!"<<std::endl;

    if(b!=NULL)std::cout<<"Next block is ["<< b->getTagX() <<", "<< b->getTagY() <<"]."<<std::endl;
    else std::cout<<"Exploration over!"<<std::endl;
    m.show();
    app.exec();
    return 0;
}


