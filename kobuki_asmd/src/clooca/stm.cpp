
#include "stm.h"



/*============================================================================
 * KobukiStateMachine class
 *
 */
const int KobukiStateMachine::matrix[] = {
KobukiStateMachine_STATE_mapping,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_changeDrection,KobukiStateMachine_STATE_stop,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_running,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_foundObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_mapping,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_decideNext,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_decideNext,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE
};

int KobukiStateMachine::execute(MEXUEvent *event) {
  if (!event) {
    return 0;
  }                                        

  int next_state = matrix[current_state * 5 + event->event_id];
    
  if(next_state == -1) {
    return 0;
    }

  printf("KobukiStateMachine: event %d, state: %d -> %d\n",
    event->event_id, current_state, next_state);
    
  current_state = next_state;
    
  switch(current_state) {
    
  case KobukiStateMachine_STATE_start: /* start */
    
    break;
    
  case KobukiStateMachine_STATE_decideNext: /* decideNext */
    // decide next block.
next_block = map.getNextBlock();
if( next_block == NULL ) goFinish();
else goNext();
    break;
    
  case KobukiStateMachine_STATE_changeDrection: /* changeDrection */
    // turn to next block.
turnAngle = map.getTurnAngle(next_block);
kobukimanager->changeDirection( 2.0, turnAngle );
    break;
    
  case KobukiStateMachine_STATE_running: /* running */
    // running to next block.
runDistance = map.getNextDistance(next_block);
kobukimanager->goStraight( 0.2, runDistance );
    break;
    
  case KobukiStateMachine_STATE_mapping: /* mapping */
    // check current block, nothing or IR.
map.setCurrentBlock(next_block);
map.setMarkBlock(next_block, BLANK);
goNext();
    break;
    
  case KobukiStateMachine_STATE_foundObstacle: /* foundObstacle */
    // if find a obstacle, check next block and go back to current block.
map.setMarkBlock(next_block, OBSTACLE);
next_block = map.getCurrentBlock();
runDistance = map.getNextDistance(next_block);
kobukimanager->goStraight( -0.2, runDistance );

    break;
    
  case KobukiStateMachine_STATE_stop: /* stop */
    
    break;
    
  }
  return 0;
}

