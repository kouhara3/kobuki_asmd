/**
 * @file /kobuki_driver/src/clooca/Graph_Algorithm.hpp
 *
 * @graph-algorithm.
 **/
#pragma once

#include <vector>

class MapGraph{
public:
  struct Node
  {
    int parent;
    int cost;
    Block* block;
    Node* right_node;
    Node* left_node;
    Node* up_node;
    Node* down_node;
  };

  std::vector<struct Node> graph;
  int size_x, size_y;
  int node_size;

  MapGraph(){}
  MapGraph( int idx_x, int idx_y ){
    setSize( idx_x, idx_y );
  }

  void setSize( int idx_x, int idx_y ){
    node_size = idx_x * idx_y;

    graph.resize( node_size );

    //Edges initialization
    for(int y = 0; y<idx_y; y++) {
      for(int x = 0; x<idx_x; x++) {
        int current = idx_x * y + x;

        if(y+1<idx_y) { graph[current].up_node = &graph[current+idx_x]; }
        else  { graph[current].up_node = NULL; }

        if(x+1<idx_x) { graph[current].right_node = &graph[current+1]; }
        else { graph[current].right_node = NULL; }

        if(y-1>=0) { graph[current].down_node = &graph[current-idx_x]; }
        else { graph[current].down_node = NULL; }

        if(x-1>=0) { graph[current].left_node = &graph[current-1]; }
        else  { graph[current].left_node = NULL; }

        graph[current].block = NULL;
        graph[current].parent = -1;
        graph[current].cost = -1;
      }
    }

    size_x = idx_x;
    size_y = idx_y;

    return;
  }

  void dijkstra( int s ){
    std::vector<bool> visited( node_size, false );

    for( int i=0; i<node_size; i++ ){
      graph[i].cost = INT_MAX;
      graph[i].parent = -1;
    }

    graph[s].cost = 0;
    graph[s].parent = s;

    while(1){
      int min_cost = INT_MAX;
      int min_node = -1;

      for( int i=0; i<node_size; i++ ){
        if( (visited[i] == false) && (graph[i].cost < min_cost)
            && (graph[i].block->getMark() == BLANK) ){
          min_cost = graph[i].cost;
          min_node = i;
        }				
      }

      if( min_cost == INT_MAX ) break;

      visited[min_node] = true;

      for( int j=0; j<4; j++ ){
        Node *n;
        if( j == 0 ) n = graph[min_node].down_node;
        if( j == 1 ) n = graph[min_node].right_node;
        if( j == 2 ) n = graph[min_node].left_node;
        if( j == 3 ) n = graph[min_node].up_node;

        if( n != NULL ){
          if( n->cost > min_cost+1 ){
            n->cost = min_cost+1;
            n->parent = min_node;
          }
        }
      }

    }
    return;
  }

  void updateShortestPath( Block* now ){
    int current = size_x * now->getTagY() + now->getTagX();
    dijkstra( current );

    return;
  }

  Block* getNearestUnknownBlock( Block* now ){
    updateShortestPath( now );

    int min_cost = INT_MAX;
    int nearest_unknown_node = -1;
    for( int i=0; i<node_size; i++ ){
      if( (graph[i].block->getMark() == UNKNOWN) && (graph[i].cost < min_cost) ){
        min_cost = graph[i].cost;
        nearest_unknown_node = i;
      }
    }

    if( nearest_unknown_node > -1 ) return( graph[nearest_unknown_node].block );
    else return( NULL );
  }

  std::vector<Block*> getPathToNearestUnknownBlock( Block* now ){
    Block* nearest_unknown_block;
    nearest_unknown_block = getNearestUnknownBlock( now );

    if( nearest_unknown_block == NULL ){
      std::vector<Block*> path;
      return( path );
    }
    int nearest_unknown_node = size_x * nearest_unknown_block->getTagY() + nearest_unknown_block->getTagX();

    std::vector<int> tmp;
    int p = nearest_unknown_node;
    while( graph[p].parent != p ){
      tmp.push_back( p );
      p = graph[p].parent;
    }

    std::vector<Block*> path;
    for( int i=tmp.size()-1; i>=0; i--){
      path.push_back( graph[tmp[i]].block );
    }

    return( path );
  }

  std::vector<Block*> getPathToTargetBlock( Block* now, Block* target ){
    updateShortestPath( target );
    int current = size_x * now->getTagY() + now->getTagX();

    if( graph[current].cost == INT_MAX ){
      std::vector<Block*> path;
      return( path );
    }

    std::vector<Block*> path;
    int p = current;
    while( graph[p].parent != p ){
      p = graph[p].parent;
      path.push_back( graph[p].block );
    }

    return( path );
  }

  Block* getSideBlock( Block* target, int side ){
    struct Node* side_node;
    int center = size_x * target->getTagX() + target->getTagY();
    switch( side ){
      case 0: side_node = graph[center].left_node; break;
      case 1: side_node = graph[center].up_node; break;
      case 2: side_node = graph[center].right_node; break;
      case 3: side_node = graph[center].down_node; break;
      default: side_node = NULL; break;
		}

    if( side_node == NULL ) return(NULL);
    else if( side_node->block->getMark() == BLANK ) return(side_node->block);
    else return(NULL);
	}

	void setBlock( int x, int y, Block* block ){
		int current = size_x * y + x;
		graph[current].block = block;
	}

};


