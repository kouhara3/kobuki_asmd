/**
 * @file /kobuki_driver/src/test/Coordinate.cpp
 *
 * @Map in coordinate.
 **/
#pragma once
/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
#include <GL/glut.h>
#include "Coordinate.cpp"
#include "KobukiManager.hpp"
#include "Block.cpp"
#include "mapgraph.hpp"
/*****************************************************************************
** Define
*****************************************************************************/
#define MINIMUM_SIZE 0.4
#define KOBUKI_SIZE 0.35

/*****************************************************************************
** Classes
*****************************************************************************/

enum Direction{
  RIGHT,
  LEFT,
  UP,
  DOWN,
};

class Map {
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
  void resize(double coord_x, double coord_y){
    this->max.setCoordinate(coord_x, coord_y);
    this->current_block = NULL;
    blockListInit();
  }

  bool blockListInit(){
    block_list.clear();
    
    int idx_x = this->max.getCoordinateX()/DEFAULT_BLOCK_LENGTH;
    int idx_y = this->max.getCoordinateY()/DEFAULT_BLOCK_LENGTH;

    std::cout << "idx_x = " << idx_x << std::endl; 
    std::cout << "idx_y = " << idx_y << std::endl;
   

    if( idx_x>0 && idx_y>0 ){
      for(int i = 0; i<idx_x; i++) {
      
        std::vector<Block> list;
      
        for(int j = 0; j<idx_y; j++) {
          Coordinate coord;
          coord.setCoordinate(i*DEFAULT_BLOCK_LENGTH-0.4f, j*DEFAULT_BLOCK_LENGTH-0.4f);
          Block block(coord, DEFAULT_BLOCK_LENGTH, i, j);
  	  list.push_back(block);
        }

        this->block_list.push_back(list);   
        list.clear(); 
      }

      // edge initialize
      mapgraph.setSize( idx_x, idx_y );
      for(int i = 0; i<idx_x; i++) {
        for(int j = 0; j<idx_y; j++) {
          mapgraph.setBlock( i, j , &this->block_list[i][j]);
        }
      }

      this->current_block = (Block*) &block_list[1][1];
      this->next_block = this->current_block;

      //block_list[1][1].setMark(BLANK);
      //block_list[1][1].setHasKobuki(true);

      std::cout << "Block list initialezed successfully." << std::endl;
      std::cout << "Size of block_list is [" << this->block_list.size() <<"]" << std::endl;
      return true;
    } else {
      this->current_block = NULL;
      this->next_block = this->current_block;
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

//////This block can provide a instant value of angle and distance to be advanced///////////
//
//    double dist_x = this->manager.getPose().x() - next_block->getCenterPoint().getCoordinateX();
//    double dist_y = this->manager.getPose().x() - next_block->getCenterPoint().getCoordinateY();
//    double angle_now = this->manager.getPose().degrees();
//
//    this->angle_to_next = (atan(dist_x/dist_y)/ecl::pi)*180 - angle_now;
//    this->distance_to_next = pow( pow(dist_x, 2.0)+pow(dist_y, 2.0), 0.5);
////////////////////////////////////////////////////////////////////////////////////////////

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

    //if((turn_angle < 5.0) && (turn_angle > -5.0)) turn_angle = 0;
    if(turn_angle>180.0) turn_angle = turn_angle - 360.0;
    if(turn_angle<-180.0) turn_angle = turn_angle + 360.0;
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
    
    for(int i = 0; i<this->block_list.size();i++)
    {
      int j = 0;
      while(this->block_list[i][j].getMark() != OBSTACLE && this->block_list[i][j].getMark() != WALL) j++;
      this->block_list[i][j].setMark(WALL);
    }

    for(int j = 0; j<this->block_list[0].size();j++)
    {
      int i = this->block_list.size();
      while(this->block_list[i][j].getMark() != OBSTACLE && this->block_list[i][j].getMark() != WALL) i--;
      this->block_list[i][j].setMark(WALL);
    }

    for(int i = this->block_list.size(); i>0; i--)
    {
      int j = this->block_list[0].size();
      while(this->block_list[i][j].getMark() != OBSTACLE && this->block_list[i][j].getMark() != WALL) j--;
      this->block_list[i][j].setMark(WALL);
    }

    for(int j = this->block_list[0].size(); j>0; j--)
    {
      int i = 0;
      while(this->block_list[i][j].getMark() != OBSTACLE && this->block_list[i][j].getMark() != WALL) i++;
      this->block_list[i][j].setMark(WALL);
    }
/*
    int j = 0;
    while(this->block_list[0][j].getMark() != OBSTACLE){
        j++;
        if( this->block_list[0].size() <= j ) break;
    }
    if( this->block_list[0].size() <= j ) return;

    std::cout << "first obstacle: 0," << j << std::endl;
    setWall(&this->block_list[0][j]);
*/
  }

  void setWall(Block* wall_block){

  	int idx_x = this->block_list.size();
	int idx_y = this->block_list[0].size();

	int tag_x = wall_block->getTagX();
	int tag_y = wall_block->getTagY();

	//if(!(tag_x > 0 && tag_y == 0))
	{
		wall_block->setMark(WALL);
    std::cout << "checked: " << tag_x << "," << tag_y << std::endl;

                Block temp_block = *wall_block;
                wall_list.push_back(temp_block);

		for(int i = tag_x -1; i<= tag_x + 1; i++)
		{
			if(i == -1) i = 0;
			for(int j = tag_y -1; j<= tag_y + 1; j++)
			{
				if(j == -1) j = 0;
				if(this->block_list[i][j].getMark() == OBSTACLE)
				{
					setWall(&this->block_list[i][j]);
				}
			}
		}
	} 
	/*else 
	{
		wall_block->setMark(WALL);
    std::cout << "last: " << tag_x << "," << tag_y << std::endl;
    Block temp_block = *wall_block;
    wall_list.push_back(temp_block);
	}
  */
  std::cout << "finish" << std::endl;
      return;
  }
  
  Block* findDockBlock()
  {
    ecl::Pose2D<double> pose;
    pose = this->manager.getPose();

    float x = pose.x();
    float y = pose.y();
    float angle = pose.heading();
    
    float dockX = x + cos(angle);
    float dockY = y + sin(angle);

    int tagX = (int)(dockX/DEFAULT_BLOCK_LENGTH) + 1 ;
    int tagY = (int)(dockY/DEFAULT_BLOCK_LENGTH) + 1 ;
    
    return &this->block_list[tagX][tagY];
  }

/*  ////////////////////////////////////
  void outputBlockList(){
    Global_Block_List = this->block_list;
    return;
  }

  void outputMax(){
    Global_Max = this->max;
    return;
  }
*/
  std::vector< std::vector<Block> > getBlockList(){
    return( this->block_list );
  }

  std::vector<Block> getWallList(){
    return( this->wall_list );
  }

  Coordinate getMax(){
    return( this->max );
  }

  /////////////////////////////////////
// method for queue

  void pushPath( std::vector<Block *> list ){
    while( !qu_path.empty() ) qu_path.pop();          // clear the queue
    for( int i=0; i<list.size(); i++) qu_path.push( list[i] );
    return;
  }

  Block *popPath(){
    Block *pt;
    if( qu_path.empty() ){
      pt = NULL;
    }
    else {
      pt = qu_path.front();
      qu_path.pop();
    }
    return pt;
  }

// private functions

  double getMovement( Block *block ){
    double distance = getNextDistance( block );
    double border = block->getSideLength() - distance;
    if( border < 0.0 ) border = 0.0;
    return border;
  }

  Direction getDirection(){
    ecl::Pose2D<double> pose = this->manager.getPose();
    double angle = pose.heading().degrees();
    if( (angle <= 45) && ( angle > -45 ) ) return(RIGHT);   // right
    if( (angle <= 135) && ( angle > 45 ) ) return(UP);   // up
    if( (angle <= -135) || ( angle > 135 ) ) return(LEFT); // left
    if( (angle <= -45) && ( angle > -135 ) ) return(DOWN);   // down
  }

  Direction updateDirection( Direction direct ){
    kobuki::SensorManager::Data sensor = getSensorData();

    if( sensor.left_bumper )
    {
      switch( direct ){
      case RIGHT:
        return(UP);
      case UP:
        return(LEFT);
      case LEFT:
        return(DOWN);
      case DOWN:
        return(RIGHT);
      default:
        break;
      }
    } else if( sensor.right_bumper )
    {
      switch( direct ){
      case RIGHT:
        return(DOWN);
      case UP:
        return(RIGHT);
      case LEFT:
        return(UP);
      case DOWN:
        return(LEFT);
      default:
        break;
      }
    } else{
      return( direct );
    }
  }
  

  void setObstacleSize( Block *block, double move, Direction direct ){
    // if block has not obstacle object, then create obstacle.
    if( block->borders == NULL ) block->borders = new Borders;

    switch( direct ){
      case RIGHT:
        block->borders->left = move;
        std::cout << "left: " << move << std::endl;
        break;
      case UP:
        block->borders->down = move;
        std::cout << "down: " << move << std::endl;
        break;
      case LEFT:
        block->borders->right = move;
        std::cout << "right: " << move << std::endl;
        break;
      case DOWN:
        block->borders->up = move;
        std::cout << "up: " << move << std::endl;
        break;
      default:
        break;
    }
  }

// functions for statemachine

  void mappingBlank(){
    current_block->setMark(BLANK);
    return;
  }

  void updateCurrent(){
    current_block->setHasKobuki(false);
    current_block = next_block;
    current_block->setHasKobuki(true);
    return;
  }

  /*==== not use queue ====//
  bool setPathToNextBlock(){
    Block* goal = getNextBlock();
    if ( goal != NULL ){
      std::vector<Block *> dummy;
      dummy.push_back( goal );
      pushPath( dummy );
      return(true);
    }
    else return(false);
  }
  //=======================*/

  //==== use queue ====//
  bool setPathToNextBlock(){
    std::vector<Block *> path = mapgraph.getPathToNearestUnknownBlock( this->current_block );
    if( path.empty() ) return( false );
    pushPath( path );
    return( true );
  }

  bool setPathToTargetBlock( Block* target ){
    std::vector<Block *> path = mapgraph.getPathToTargetBlock( this->current_block, target );
    if( path.empty() ) return( false );
    pushPath( path );
    return( true );
  }

  bool isReached(){
    return( qu_path.empty() );
  }

  void setNext(){
    this->next_block = popPath();
    return;
  }

  void turnToNext( double speed ){
    double turnAngle = getTurnAngle( this->next_block );
    manager.changeDirection( speed, turnAngle );
  }

  void runToNext( double speed ){
    double runDistance = getNextDistance( this->next_block );
    manager.goStraight( speed, runDistance );
  }

  void foundObstacle(){
    this->next_block->setMark(OBSTACLE);
    double movement = getMovement( this->next_block );
    Direction direction = getDirection();
    Direction new_direction = updateDirection( direction );
    if( direction != new_direction ) movement = 0.10;
    setObstacleSize( this->next_block, movement, direction );
  }

  void goBackToCurrent( double speed ){
    this->next_block = this->current_block;
    double runDistance = getNextDistance( this->next_block );
    manager.goStraight( -speed, runDistance );
  }

  void goBack( double speed, double distance ){
    manager.goStraight( -speed, distance );
  }

  void recordIR( void ){
    manager.getIRData();
    for( unsigned int i = 0; i < 3; i++ ){
    
      kobuki::IRManager::Data ir_data = manager.getIRData();

      if(/*  (ir_data[i].far_left     == 1)
        || (ir_data[i].far_center   == 1)
        || (ir_data[i].far_right    == 1)
        ||*/ (ir_data[i].near_left    == 1)
        || (ir_data[i].near_center  == 1)
        || (ir_data[i].near_right   == 1) ){
        
        current_block->setIRMark(EXIST);
        break;
      }
    }
  }
  
  Block* searchIRBlock( void ){
    for( unsigned int i = 0; i < this->block_list.size(); i++ ){
      for( unsigned int j = 0; j < this->block_list[i].size(); j++ ){
        if( this->block_list[i][j].getIRMark() == EXIST ){
          return( &this->block_list[i][j] );
        }
      }
    }
    return( NULL );
  }

  /*==== not use queue ====//
  bool setNextIRBlock( void ){
    Block *ir = map.searchIRBlock();
    if( ir == MULL ) return(false);             // IRmark not found    
    
    std::vector<Block *> dummy;
    dummy.push_back( ir );
    map.pushPath( dummy );
    return(true);
  }
  //=======================*/

  //==== use queue ====//
  bool setNextIRBlock( void ){
    Block *ir = NULL;

    while(1){
      ir = searchIRBlock();
      if( ir == NULL ) break;             // IRmark not found

      if( setPathToTargetBlock(ir) ) break;   // set path
      else ir->setIRMark( NOT_EXIST );    // cannot reach the block
    }
    
    if( ir != NULL ) return(true);
    else return(false);
  }
  
  void clearIRMark( void ){
    std::queue<Block *> q;
    q.push( this->current_block );
    while( !qu_path.empty() ){
      Block* tmp = q.front();
      q.pop();

      if( tmp->getIRMark() == EXIST ){
        tmp->setIRMark( NOT_EXIST );

        int x = tmp->getTagX();
        int y = tmp->getTagY();
        int max_x = this->block_list.size()-1;
        int max_y = this->block_list[x].size()-1;
        
        for( int i = x-1; i <= x+1; i++ ){
          if( (i < 0) || (i > max_x) ) continue;

          for( int j = y-1; j <= y+1; j++ ){
            if( (j < 0) || (j > max_y) ) continue;

            if( this->block_list[i][j].getIRMark() == EXIST )
              q.push( &this->block_list[i][j] );
          }
        }
      }
    }
    return;
  }

  void docking( void ){
    manager.docking();
    return;
  }

  void Wait( double t ){
    time_t start;
    time_t now;

    time(&start);
    time(&now);
    while( (now-start) < t ){
      time(&now);
    }
    return;
  }

  void markDockBlock( Block* dock ){
    std::queue<Block*> qu_dock;

    dock->setMark( DOCK );
    qu_dock.push( dock );

    while( !qu_dock.empty() ){
      Block* tmp = qu_dock.front();
      qu_dock.pop();

      int x = tmp->getTagX();
      int y = tmp->getTagY();
      int max_x = this->block_list.size()-1;
      int max_y = this->block_list[x].size()-1;
        
      for( int i = x-1; i <= x+1; i++ ){
        if( (i < 0) || (i > max_x) ) continue;

        for( int j = y-1; j <= y+1; j++ ){
          if( (j < 0) || (j > max_y) ) continue;

          if( this->block_list[i][j].getMark() == OBSTACLE ){
            tmp->setMark( DOCK );
            qu_dock.push( &this->block_list[i][j] );
          }
        }
      }
    }
    return;
  }
  
  //
  void searchObstacleBlocks(){
    for( unsigned int i = 0; i < this->block_list.size(); i++ ){
      for( unsigned int j = 0; j < this->block_list[i].size(); j++ ){
        if( this->block_list[i][j].getMark() == OBSTACLE ){
          qu_obstacle.push( &this->block_list[i][j] );
        }
      }
    }
    return;
  }

  bool setNextUncheckedObstacle(){
    if( qu_obstacle.empty() ){
      this->next_obstacle = NULL;
      return(false);
    }
    else {
      this->next_obstacle = qu_obstacle.front();
      qu_obstacle.pop();
      flg_border = 0;
      return(true);
    }
  }

  Block* getUnmeasuredSideBlock( Block* obstacle ){
    Block* side_block = NULL;

    while( this->flg_border < 4 ){
      double tmp;
      switch( this->flg_border ){
        case 0: tmp = obstacle->borders->left; break;
        case 1: tmp = obstacle->borders->up; break;
        case 2: tmp = obstacle->borders->right; break;
        case 3: tmp = obstacle->borders->down; break;
        default: tmp = -1.0; break;
      }
      if( tmp == 0.0 ){
        side_block = mapgraph.getSideBlock( obstacle, this->flg_border );
        if( side_block != NULL ) break;
      }  
       this->flg_border++;
    }

    return( side_block );
  }

  bool setNextSide(){
    Block* next_side = getUnmeasuredSideBlock( next_obstacle );
    if( next_side == NULL ) {
      return(false);
    }
    else{
      if( setPathToTargetBlock(next_side) ) return(true);
      else(false);
    }
  }

  void turnToObstacle( double speed ){
    double turnAngle = getTurnAngle( this->next_obstacle );
    manager.changeDirection( speed, turnAngle );
  }

  void runToObstacle( double speed ){
    double runDistance = getNextDistance( this->next_obstacle );
    manager.goStraight( speed, runDistance );
  }

  void measureBorder(){
    double movement = getMovement( this->next_obstacle );
    Direction direction = getDirection();
    setObstacleSize( this->next_obstacle, movement, direction );
    this->flg_border++;
  }

  // get method for eventmanager

  kobuki::SensorManager::Data getSensorData(){
    return( manager.getSensorData() );
  }

  kobuki::IRManager::Data getIRData(){
    return( manager.getIRData() );
  }

  KobukiManager::RunData getRunData(){
    return( manager.getRunData() );
  }

  void showMe( void ){
    for( unsigned int i = 0; i < this->block_list.size(); i++ ){
      for( unsigned int j = 0; j < this->block_list[i].size(); j++ ){
        if( this->block_list[i][j].getMark() == BLANK ) std::cout << " ";
        else if( this->block_list[i][j].getMark() == UNKNOWN ) std::cout << "$";
        else if( this->block_list[i][j].getMark() == OBSTACLE ) std::cout << "*";
        else if( this->block_list[i][j].getMark() == WALL ) std::cout << "#";
      }
      std::cout << std::endl;
    }
    return;
  }

  void showIR( void ){
    for( unsigned int i = 0; i < this->block_list.size(); i++ ){
      for( unsigned int j = 0; j < this->block_list[i].size(); j++ ){
        if( this->block_list[i][j].getIRMark() == NOT_EXIST ) std::cout << " ";
        else if( this->block_list[i][j].getIRMark() == EXIST ) std::cout << "!";
      }
      std::cout << std::endl;
    }
    return;
  }

//private:
  Coordinate max;
  std::vector< std::vector<Block> > block_list;
  std::vector< Block > wall_list;
  Block* current_block;
  Block* next_block;
  KobukiManager manager;
  std::queue<Block *> qu_path;
  MapGraph mapgraph;
  double angle_to_next;
  double distance_to_next;
  std::queue<Block *> qu_obstacle;
  Block* next_obstacle;
  int flg_border;

 };

/*****************************************************************************
** Test Main
*****************************************************************************/

/*
int main(int argc, char* argv[]) {

    Map m(120,200);
    m.blockListInit();
    m.getMapInfo();
    Block* b = m.getNextBlock();
    if(m.getCurrentBlock()!=NULL)std::cout<<"Kobuki is now at block ["<< m.getCurrentBlock()->getTagX() <<","<< m.getCurrentBlock()->getTagY() <<"]."<<std::endl;
    else std::cout<<"Null block position!"<<std::endl;

    if(b!=NULL)std::cout<<"Next block is ["<< b->getTagX() <<", "<< b->getTagY() <<"]."<<std::endl;
    else std::cout<<"Exploration over!"<<std::endl;

    return 0;
}


*/
