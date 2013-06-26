#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include <stdio.h>

#define MEXU_MAX_INSTANCES 10
#define MEXU_MSEC_PER_TICK 50

class StateMachine;
class EventManager;
class MEXUTimerEventManager;

extern EventManager eventManager;
extern MEXUTimerEventManager timer;

class MEXUEvent {
public:
    int event_id;
    StateMachine *target;
    MEXUEvent *next;
    MEXUEvent(StateMachine *_target, int _id);
};

class MEXUEventQueue {
protected:    
    MEXUEvent *head;
public:    
    MEXUEvent *deq();
    void enq(MEXUEvent *new_e);
};

class MEXUObject {
private:
    static MEXUObject *_instances[MEXU_MAX_INSTANCES];
    static int _instances_num;
public:
    MEXUObject();
    static int instances_num (void);
    static MEXUObject **instances(void);
};

class StateMachine : public MEXUObject {
protected:
    int current_state;
public:
    StateMachine();
    ~StateMachine(){}
    void sendEvent(int event_id);
    virtual int execute(MEXUEvent *e) = 0;
};

class EventManager {
private:
    int stm_len;
    StateMachine *stms[16];
    MEXUEventQueue que;
public:
    EventManager();
    void addSTM(StateMachine *stm);
    void execute();
    void sendEvent(StateMachine *target, int event_id);
    void sendEvent (MEXUEvent *e);
};

class MEXUTimerEvent : public MEXUEvent {
public:
    MEXUTimerEvent *next;
    int remaining; /* in millisecond */
    MEXUTimerEvent(int duration, StateMachine *_target, int _id);
};

class MEXUTimerEventManager {
protected:    
    MEXUTimerEvent *head; /* sorted */
public:
    void tick(void);
    void add(int duration, StateMachine *target, int event_id);
};

extern void mexu_lock();
extern void mexu_unlock();

#endif
