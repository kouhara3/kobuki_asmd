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
#include <vector>
#include "Coordinate.cpp"
#include "KobukiManager.cpp"
#include "Block.cpp"
/*****************************************************************************
** Define
*****************************************************************************/
#define MINIMUM_SIZE 50
/*****************************************************************************
** Classes
*****************************************************************************/

class Map {
public:
  Map(){
    this->max.setCoordinate(0,0);
    this->current_block = NULL;
    //this->KobukiManager[0] = KobukiManager();
    //this->KobukiManager[1] = KobukiManager();
  }
  Map(int coord_x, int coord_y){
    this->max.setCoordinate(coord_x, coord_y);
    this->current_block = NULL;
  }
  void resize(Coordinate coord){
    this->max = coord;
    this->current_block = NULL;
  }

  bool blockListInit(){

    Coordinate coord;
    int idx_x = this->max.getCoordinateX()/DEFAULT_BLOCK_LENGTH;
    int idx_y = this->max.getCoordinateY()/DEFAULT_BLOCK_LENGTH;
    std::cout << "idx_x = " << idx_x << std::endl; 
    std::cout << "idx_y = " << idx_y << std::endl;
   

    if( idx_x>0 && idx_y>0 ){
      for(int i = 0; i<idx_x; i++) {
      
        std::vector<Block> list;
      
        for(int j = 0; j<idx_y; j++) {
          coord.setCoordinate(20 + i*DEFAULT_BLOCK_LENGTH, 20 + j*DEFAULT_BLOCK_LENGTH);
          Block block(coord, DEFAULT_BLOCK_LENGTH, i, j);
  	  list.push_back(block);
        }

        this->block_list.push_back(list);   
        list.clear(); 
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

    int now_x = this->current_block->getTagX();
    int now_y = this->current_block->getTagY();
    
    if( now_y-1>=0 && this->block_list[now_x][now_y-1].isUnkonwn() ){
      return (&this->block_list[now_x][now_y-1]);
    } else if( now_x+1<this->block_list.size() && this->block_list[now_x+1][now_y].isUnkonwn()){
      return (&this->block_list[now_x+1][now_y]);
    } else if( now_x-1>=0 && this->block_list[now_x-1][now_y].isUnkonwn()){
      return (&this->block_list[now_x-1][now_y]);
    } else if( now_y+1< this->block_list[now_x].size() && !this->block_list[now_x][now_y+1].isObstacle()){
      return (&this->block_list[now_x][now_y+1]);
    } else if( now_x-1>=0 ){
      return (&this->block_list[now_x-1][now_y]);
    } else{
      return NULL;
    }

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
    return;
  }
private:
  Coordinate max;
  std::vector< std::vector<Block> > block_list;
  Block* current_block;
  //KobukiManager[2] Manager;
 };

/*****************************************************************************
** Test Main
*****************************************************************************/

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


