#include "MEXU.h"

/* Machine Independent Part of MEXU runtime */

MEXUEvent::MEXUEvent(StateMachine *_target, int _id){
    target   = _target;
    event_id = _id;
}

MEXUObject *MEXUObject::_instances[MEXU_MAX_INSTANCES];
int MEXUObject::_instances_num = 0;

MEXUObject::MEXUObject() {
    _instances[_instances_num++] = this;
}

int MEXUObject::instances_num (void){
    return _instances_num; 
}

MEXUObject** MEXUObject::instances(void) {
    return _instances;
}

StateMachine::StateMachine() {
    current_state = 0;
}

void StateMachine::sendEvent(int event_id) {
    eventManager.sendEvent(this, event_id);
}

MEXUEvent *MEXUEventQueue::deq() {
    if(head == NULL) return NULL;
    MEXUEvent *p = head;
    head = p->next;
    p->next = NULL;
    return p;
}

void MEXUEventQueue::enq(MEXUEvent *new_e) {
    if(head == NULL) {
        head = new_e;
        head->next = NULL;
    }else{
        MEXUEvent *e = head;
        while(e->next != NULL){
            e = e->next;
        }
        e->next     = new_e;
        new_e->next = NULL;
    }
}

EventManager::EventManager() {
    stm_len = 0;
}

void EventManager::addSTM(StateMachine *stm) {
    stms[stm_len] = stm;
    stm_len++;
}

void EventManager::execute() {
    while (MEXUEvent *e = que.deq ()) {
        e->target->execute (e);
        delete e;
    }
}

void EventManager::sendEvent (MEXUEvent *e) {
    que.enq (e);
}

void EventManager::sendEvent (StateMachine *target, int event_id) {
    que.enq (new MEXUEvent (target, event_id));
}

MEXUTimerEvent::MEXUTimerEvent(int duration, StateMachine *_target, int _id) 
        : MEXUEvent(_target, _id) {
    remaining = duration;
}

void MEXUTimerEventManager::tick(void) {
    mexu_lock ();
    
    MEXUTimerEvent *current = head;
    while(current) {
        current->remaining -= MEXU_MSEC_PER_TICK;
        if (current->remaining <= 0) {
            head = head->next;               
            eventManager.sendEvent(current); /* should it insert first? */
        }
        current = current->next;
    }

    mexu_unlock();
}

void MEXUTimerEventManager::add(int duration, StateMachine *target, int event_id) {
    MEXUTimerEvent *e = new MEXUTimerEvent(duration, target, event_id);

    mexu_lock ();

    if(head == NULL) {
        head = e;
        head->next = NULL;
    }else if(head->remaining > e->remaining){
        /* キューの先頭よりも短いタイマをつなげる場合 */
        e->next = head;
        head = e;
    }else if(head->next != NULL){
        /* キューに2つ以上つながっている場合 */
        MEXUTimerEvent *current = head;
        while(current->next) {
            if (current->next->remaining > e->remaining) {
                // insert
                e->next = current->next;
                current->next = e;
                break;
            }
            current = current->next;
        }
        /* 一番長いタイマの場合，最後につなげる */
        current->next = e;
        e->next = NULL;
    }else{
        /* キューに1つしかつながっていない場合 */
        head->next = e;
        e->next = NULL;
    }

    mexu_unlock ();
}