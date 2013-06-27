
#ifndef D_MEXU_H_
#define D_MEXU_H_

#include "main.h"
#include "MEXU.h"
#include "Map.cpp"

/*====================================================
 * tmp class
 *
 */

class tmp : public StateMachine {
private:
  static const int matrix[];
public:
  /* attributes */
  int current_state;

  int val;

  /* methods */                          
  tmp() : StateMachine(){
      eventManager.addSTM(this);
      current_state = 0;
  }

  ~tmp() {}
  int execute(MEXUEvent *event);

  void init(void) {
  val = 0;
  }

};
/* State ID Definitions */
#define tmp_STATE_IGNORE -1
  

#define tmp_STATE_stone 0

#define tmp_STATE_scissors 1

#define tmp_STATE_paper 2


/* Event ID Definitions */

#define tmp_EVENT_start 0

#define tmp_EVENT_win1 1

#define tmp_EVENT_win2 2

#define tmp_EVENT_win3 3




/*====================================================
 * KobukiStateMachine class
 *
 */

class KobukiStateMachine : public StateMachine {
private:
  static const int matrix[];
public:
  /* attributes */
  int current_state;
  Map map;
  KobukiManager* kobukimanager;

  /* methods */                          
  KobukiStateMachine() : StateMachine(){
      eventManager.addSTM(this);
      current_state = 0;
      kobukimanager = map.getKobukiManager();
  }

  ~KobukiStateMachine() {}
  int execute(MEXUEvent *event);

};
/* State ID Definitions */
#define KobukiStateMachine_STATE_IGNORE -1
  

#define KobukiStateMachine_STATE_wait 0

#define KobukiStateMachine_STATE_run1 1

#define KobukiStateMachine_STATE_turnRight 2

#define KobukiStateMachine_STATE_run2 3

#define KobukiStateMachine_STATE_turnLeft 4


/* Event ID Definitions */

#define KobukiStateMachine_EVENT_start 0

#define KobukiStateMachine_EVENT_win1 1

#define KobukiStateMachine_EVENT_win2 2

#define KobukiStateMachine_EVENT_win3 3

#define KobukiStateMachine_EVENT_BUMPER_PRESSED 4

#define KobukiStateMachine_EVENT_RUN_REACHED 5

#define KobukiStateMachine_EVENT_TURN_REACHED 6



#endif /* D_MEXU_H_ */
