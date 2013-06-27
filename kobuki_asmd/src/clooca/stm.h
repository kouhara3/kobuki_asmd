
#ifndef D_MEXU_H_
#define D_MEXU_H_

#include "main.h"
#include "MEXU.h"


/* State ID Definitions */
#define KobukiStateMachine_STATE_IGNORE -1
  

#define KobukiStateMachine_STATE_start 0

#define KobukiStateMachine_STATE_decideNext 1

#define KobukiStateMachine_STATE_changeDrection 2

#define KobukiStateMachine_STATE_running 3

#define KobukiStateMachine_STATE_mapping 4

#define KobukiStateMachine_STATE_foundObstacle 5

#define KobukiStateMachine_STATE_stop 6


/* Event ID Definitions */

#define KobukiStateMachine_EVENT_BUMPER_PRESSED 0

#define KobukiStateMachine_EVENT_NEXT 1

#define KobukiStateMachine_EVENT_FINISH 2

#define KobukiStateMachine_EVENT_TURN_REACHED 3

#define KobukiStateMachine_EVENT_RUN_REACHED 4



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

  Block* next_block;

  double turnAngle;

  double runDistance;

  //std::vector<std::string> statename;
  //std::vector<std::string> eventname;

  /* methods */                          
  KobukiStateMachine() : StateMachine(){
      eventManager.addSTM(this);
      current_state = 0;

    map.resize(2.5, 2.5);
	  kobukimanager = map.getKobukiManager();
	  next_block = map.getCurrentBlock();
	  turnAngle = 0;
	  runDistance = 0;
  //statename.resize(7);
  //eventname.resize(5);
  //statename = { "start", "decideNext", "changeDirection", "running", "mapping", "foundObstacle", "stop" };
  //eventname = { "BumperPressed", "Next", "Finish", "TurnReached", "RunReached" };
  }

  ~KobukiStateMachine() {}
  int execute(MEXUEvent *event);

  void goNext() {
  this->sendEvent( KobukiStateMachine_EVENT_NEXT );
  }

  void goFinish() {
  this->sendEvent( KobukiStateMachine_EVENT_FINISH );
  }

};

#endif /* D_MEXU_H_ */
