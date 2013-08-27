/**
 * @file /kobuki_driver/src/test/Coordinate.cpp
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
#include <queue>
#include <GL/glut.h>
#include "Coordinate.cpp"
#include "KobukiManager.hpp"
#include "Block.cpp"
#include "mapgraph.hpp"
/*****************************************************************************
** Define
*****************************************************************************/
#define MINIMUM_SIZE 0.5
#define KOBUKI_SIZE 0.35
#define BLOCK_SIZE 50
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
          coord.setCoordinate(i*DEFAULT_BLOCK_LENGTH, j*(DEFAULT_BLOCK_LENGTH-0.04));
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

      this->current_block = (Block*) &block_list[0][0];
      this->next_block = this->current_block;
      block_list[0][0].setMark(BLANK);
      block_list[0][0].setHasKobuki(true);
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

    if((turn_angle < 5.0) && (turn_angle > -5.0)) turn_angle = 0;
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
  
  ////////////////////////////////////
  void bmp_file(GLint WindowWidth, GLint WindowHeight) {

	glReadBuffer(GL_FRONT);

	FILE* pDummyFile;
	FILE* pWritingFile;
	GLubyte* pPixelData;
	GLubyte BMP_Header[BMP_Header_Length]; 
	GLint i, j; 
	GLint PixelDataLength; 
	
	// 计算像素数据的实际长度
	i = WindowWidth * 3;		// 得到每一行的像素数据长度 
	while( i%4 != 0 )		// 补充数据，直到i是的倍数 
		++i; 
	PixelDataLength = i * WindowHeight; 
	
	// 分配内存和打开文件 
	pPixelData = (GLubyte*)malloc(PixelDataLength); 
	if( pPixelData == 0 ) 
		exit(0); 
	fopen_s(&pDummyFile,"dummy.bmp", "rb+");
	if( pDummyFile == 0 ) 
		exit(0); 
	fopen_s(&pWritingFile,"Map.bmp", "wb+"); 
	if( pWritingFile == 0 )
		exit(0);
	
	// 读取像素
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glReadPixels(0, 0, WindowWidth, WindowHeight, GL_RGB, GL_UNSIGNED_BYTE, pPixelData); 
	// 把dummy.bmp的文件头复制为新文件的文件头
	fread(BMP_Header, sizeof(BMP_Header), 1, pDummyFile);
	fwrite(BMP_Header, sizeof(BMP_Header), 1, pWritingFile);
	fseek(pWritingFile, 0x0012, SEEK_SET);
	
	i = WindowWidth; 
	j = WindowHeight;
	fwrite(&i, sizeof(i), 1, pWritingFile);
	fwrite(&j, sizeof(j), 1, pWritingFile);
	
	// 写入像素数据 
	fseek(pWritingFile, 0, SEEK_END); 
	fwrite(pPixelData, PixelDataLength, 1, pWritingFile); 
	
	// 释放内存和关闭文件 
	fclose(pDummyFile);
	fclose(pWritingFile);
	free(pPixelData);
	
	printf("Picture was saved as grap.bmp\n");
  }

  static void key_callback(unsigned char key, int x, int y)
  {
        switch(key) {
		case 'q':
			exit(0);
			break;
		case 's':
                        float width = this->max.getCoordinateX()*100;
       			float height = this->max.getCoordinateY()*100;
			bmp_file((GLint)width, (GLint)height);		
			break;
		default:
			break;
	}
	glutPostRedisplay(); 
  }

  void makeMap(){

    float width = this->max.getCoordinateX()*100/2;
    float height = this->max.getCoordinateY()*100/2;
    float block_width =  BLOCK_SIZE/width;
    float block_height =  BLOCK_SIZE/height;
    int idx_x = this->block_list.size();
    int idx_y = this->block_list[0].size();
    
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(1.0f, 1.0f ,1.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 1.0f);
    for(int i = 0; i<idx_x; i++){
      float pointX = -1.0f + i * block_width;
      for(int j = 0; j<idx_y ; j++){ 
          float pointY = -1.0f + j*block_height;
          glEnable(GL_LINE_STIPPLE);
          glColor3f(0.0f, 0.0f, 0.0f);
    	  glLineStipple(1, 0xAAAA);
          
	  glBegin(GL_POLYGON); 
	    glVertex2f(pointX, pointY); 
            glVertex2f(pointX + block_width, pointY); 
            glVertex2f(pointX + block_width, pointY + block_height); 
            glVertex2f(pointX, pointY + block_height); 
          glEnd();
          
          glDisable(GL_LINE_STIPPLE);

          if(this->block_list[i][j].getMark() == OBSTACLE )
          {
            
          }
          if(this->block_list[i][j].getMark() == WALL)
          {
            
          }
        }
      }
    }
  }

  int showMap(int argc, char * argv[]){
    int width = this->max.getCoordinateX()*100;
    int height = this->max.getCoordinateY()*100;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE);
    glutInitWindowSize(weight, height);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Mapping!");

    glutDisplayFunc(&makeMap);
    glutKeyboardFunc(&key_callback);

    glutMainLoop();
    return 0;
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
    double move = getNextDistance( block );
    move += ( KOBUKI_SIZE / 2 ) - ( block->getSideLength() / 2 );
    return move;
  }

  Direction getDirection(){
    ecl::Pose2D<double> pose = this->manager.getPose();
    double angle = pose.heading().degrees();
    if( (angle <= 45) && ( angle > -45 ) ) return(RIGHT);   // right
    if( (angle <= 135) && ( angle > 45 ) ) return(UP);   // up
    if( (angle <= -135) || ( angle > 135 ) ) return(LEFT); // left
    if( (angle <= -45) && ( angle > -135 ) ) return(DOWN);   // down
  }

  void setObstacleSize( Block *block, double move, Direction direct ){
    // if block has not obstacle object, then create obstacle. 
    if( block->borders == NULL ) block->borders = new Borders;
    switch( direct ){
      case RIGHT:
        block->borders->left = move;
        break;
      case UP:
        block->borders->down = move;
        break;
      case LEFT:
        block->borders->right = move;
        break;
      case DOWN:
        block->borders->up = move;
        break;
      default:
        break;
    }
  }

// functions for statemachine

  void updateCurrent(){
    current_block->setHasKobuki(false);
    current_block = next_block;
    current_block->setMark(BLANK);
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
    setObstacleSize( this->next_block, movement, direction );
  }

  void goBackToCurrent( double speed ){
    this->next_block = this->current_block;
    double runDistance = getNextDistance( this->next_block );
    manager.goStraight( -speed, runDistance );
  }


  void recordIR( void ){
    manager.getIRData();
    for( unsigned int i = 0; i < 3; i++ ){
    
      kobuki::IRManager::Data ir_data = manager.getIRData();

      if(  (ir_data[i].far_left     == 1)
        || (ir_data[i].far_center   == 1)
        || (ir_data[i].far_right    == 1)
        || (ir_data[i].near_left    == 1)
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
        int x = tmp->getTagX();
        int y = tmp->getTagY();
        int max_x = this->block_list.size()-1;
        int max_y = this->block_list[x].size()-1;
        if(( x > 0 )&&( this->block_list[x-1][y].getIRMark() == EXIST ))
          q.push( &this->block_list[x-1][y] );
        if(( x < max_x )&&( this->block_list[x+1][y].getIRMark() == EXIST ))
          q.push( &this->block_list[x+1][y] );
        if(( y > 0 )&&( this->block_list[x][y-1].getIRMark() == EXIST ))
          q.push( &this->block_list[x][y-1] );
        if(( y < max_y )&&( this->block_list[x][y+1].getIRMark() == EXIST ))
          q.push( &this->block_list[x][y+1] );

        tmp->setIRMark( NOT_EXIST );
      }
    }
    return;
  }

  void docking( void ){
    manager.docking();
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


//private:
  Coordinate max;
  std::vector< std::vector<Block> > block_list;
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
