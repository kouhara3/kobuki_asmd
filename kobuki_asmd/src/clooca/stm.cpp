
#include "stm.h"



/*============================================================================
 * tmp class
 *
 */
const int tmp::matrix[] = {
tmp_STATE_IGNORE,tmp_STATE_scissors,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_paper,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_stone,tmp_STATE_IGNORE,tmp_STATE_IGNORE,tmp_STATE_IGNORE
};

int tmp::execute(MEXUEvent *event) {
  if (!event) {
    return 0;
  }                                        

  int next_state = matrix[current_state * 7 + event->event_id];
    
  if(next_state == -1) {
    return 0;
    }

  printf("tmp: event %d, state: %d -> %d\n",
    event->event_id, current_state, next_state);
    
  current_state = next_state;

  return 0;
}


/*============================================================================
 * KobukiStateMachine class
 *
 */
const int KobukiStateMachine::matrix[] = {
KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_run,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_turn,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_run
};

int KobukiStateMachine::execute(MEXUEvent *event) {
  if (!event) {
    return 0;
  }                                        

  int next_state = matrix[current_state * 7 + event->event_id];
    
  if(next_state == -1) {
    return 0;
    }

  printf("KobukiStateMachine: event %d, state: %d -> %d\n",
    event->event_id, current_state, next_state);
    
  current_state = next_state;
    
  switch(current_state) {
    
  case KobukiStateMachine_STATE_wait: /* wait */
    
    break;
    
  case KobukiStateMachine_STATE_run: /* run */
    kobukimanager.goStraight( 0.3, 1.0);
    
    break;
    
  case KobukiStateMachine_STATE_turn: /* turn */
    kobukimanager.changeDirection( 1.3, 180);

    break;
    
  }
  return 0;
}

