
#include "stm.h"





/*============================================================================
 * KobukiStateMachine class
 *
 */
const int KobukiStateMachine::matrix[] = {
KobukiStateMachine_STATE_mapping,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextBlock,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_1,KobukiStateMachine_STATE_initializeMap,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_turnToNext_1,KobukiStateMachine_STATE_mapping,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_runToNext_1,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_1,KobukiStateMachine_STATE_foundObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextBlock,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_saveMap,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextIRBlock,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextSide,KobukiStateMachine_STATE_saveMap,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_2,KobukiStateMachine_STATE_setNextObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_turnToNext_2,KobukiStateMachine_STATE_turnToObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_runToNext_2,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_2,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextSide,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_runToObstacle,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_measureBorder,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_3,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_turnToNext_3,KobukiStateMachine_STATE_docking,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_runToNext_3,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkNext_3,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_checkDock,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_stop,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_setNextIRBlock,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE,KobukiStateMachine_STATE_IGNORE
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
  this->action(current_state);
  return 0;
}

void KobukiStateMachine::action(int state) {
  switch(state) {
    
  case KobukiStateMachine_STATE_start: /* start */
    // 開始用ダミー状態
    break;
    
  case KobukiStateMachine_STATE_mapping: /* mapping */
    // 現在のBlockをBLANKに設定
mapping();
    break;
    
  case KobukiStateMachine_STATE_setNextBlock: /* setNextBlock */
    // 未探索Blockを目標地点に登録
// 未探索BlockがなければFinish
setNextBlock();
    break;
    
  case KobukiStateMachine_STATE_checkNext_1: /* checkNext_1 */
    // Queueを見て移動先を設定
// 目標地点に到着したらFinish
checkNext();
    break;
    
  case KobukiStateMachine_STATE_turnToNext_1: /* turnToNext_1 */
    // 移動先の方向を向く
turnToNext();
    break;
    
  case KobukiStateMachine_STATE_runToNext_1: /* runToNext_1 */
    // 移動先に向かって進む
runToNext();
    break;
    
  case KobukiStateMachine_STATE_foundObstacle: /* foundObstacle */
    // 障害物発見したとき
// 移動先BlockをOBSTACLEに設定
// もとのBlockまでバック
foundObstacle();
    break;
    
  case KobukiStateMachine_STATE_initializeMap: /* initializeMap */
    // 地図作成の準備
// 壁の判定もする
initializeMap();
    break;
    
  case KobukiStateMachine_STATE_saveMap: /* saveMap */
    // 地図の描画，保存
saveMap();
    break;
    
  case KobukiStateMachine_STATE_setNextObstacle: /* setNextObstacle */
    // 未計測の障害物Blockを探す
// あれば計測対象に設定してNext
// なければFinish
setNextObstacle();
    break;
    
  case KobukiStateMachine_STATE_setNextSide: /* setNextSide */
    // 未探索の側面を探す
// あれば側面のBlockを移動先に指定してNext
// なければFinish
setNextSide();
    break;
    
  case KobukiStateMachine_STATE_checkNext_2: /* checkNext_2 */
    // 側面のBlockまで移動する
checkNext();
    break;
    
  case KobukiStateMachine_STATE_turnToNext_2: /* turnToNext_2 */
    // 回転
turnToNext();
    break;
    
  case KobukiStateMachine_STATE_runToNext_2: /* runToNext_2 */
    // 直進
runToNext();
    break;
    
  case KobukiStateMachine_STATE_measureBorder: /* measureBorder */
    // 障害物の大きさを記録する
// もとのBlockまでバック
measureBorder();
    break;
    
  case KobukiStateMachine_STATE_turnToObstacle: /* turnToObstacle */
    // 障害物Blockの方向を向く
turnToObstacle();
    break;
    
  case KobukiStateMachine_STATE_runToObstacle: /* runToObstacle */
    // 障害物に向かって直進
runToObstacle();
    break;
    
  case KobukiStateMachine_STATE_setNextIRBlock: /* setNextIRBlock */
    // 赤外線のあるBlockを探す
// 移動先に指定する
setNextIRBlock();
    break;
    
  case KobukiStateMachine_STATE_checkNext_3: /* checkNext_3 */
    // 指定先に移動
checkNext();
    break;
    
  case KobukiStateMachine_STATE_turnToNext_3: /* turnToNext_3 */
    // 回転
turnToNext();
    break;
    
  case KobukiStateMachine_STATE_runToNext_3: /* runToNext_3 */
    //直進
runToNext();
    break;
    
  case KobukiStateMachine_STATE_docking: /* docking */
    // 充電器とドッキング
docking();
    break;
    
  case KobukiStateMachine_STATE_checkDock: /* checkDock */
    // 充電器が2個目ならFinish
// 充電器から出ている範囲の赤外線フラグを削除
checkDock();
    break;
    
  case KobukiStateMachine_STATE_stop: /* stop */
    stop();
    break;
    
  }
}


