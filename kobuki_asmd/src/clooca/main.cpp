#define _MAIN

#include <sys/time.h>
#include <signal.h>

#include "stm.h"
#include "stmEventManager.hpp"
#include "SaveMap.cpp"

EventManager eventManager;
MEXUTimerEventManager timer;

/* Timer driver for Unix
 *
 */

static sigset_t timer_block_sigset;
static sigset_t timer_old_sigset;

void timerCallback (int num) {
	timer.tick ();
}

void timerSetup (void) {
	// setup signal
 	sigemptyset(&timer_block_sigset);
    sigemptyset(&timer_old_sigset);
    sigaddset(&timer_block_sigset, SIGALRM);

    mexu_unlock();

    // setup timer itsself
	struct itimerval new_timeset;

	signal(SIGALRM, timerCallback);

	new_timeset.it_interval.tv_sec  = 0;
	new_timeset.it_interval.tv_usec = MEXU_MSEC_PER_TICK * 1000;

	new_timeset.it_value.tv_sec  = 0;
	new_timeset.it_value.tv_usec = MEXU_MSEC_PER_TICK * 1000;

	setitimer(ITIMER_REAL, &new_timeset, NULL);
}

void mexu_lock() {
	sigprocmask(SIG_BLOCK, &timer_block_sigset, &timer_old_sigset);
}

void mexu_unlock() {
	sigprocmask(SIG_SETMASK, &timer_old_sigset, 0);
}

// signal handler

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}


/* Main function
 *
 */

int main (int argc, char *argv[])
{
	KobukiStateMachine stm;
	STMEventManager event;

  signal(SIGINT, signalHandler);

  std::cout << "Team ASMD: Demo Start." << std::endl;
  //std::cout << argv[0] << std::endl;
  //KobukiManager kobuki_manager;

  ecl::Sleep sleep(0.1);
  ecl::Pose2D<double> pose;

  while ( !stm.endflg ){
    event.checkEvent( stm );
    eventManager.execute();
    sleep();
    //pose = stm.kobukimanager->getPose();
    //std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
  }
  
  outputBlockList( stm.map );
  outputWallList( stm.map );
  outputMax( stm.map );
  showMap(argc, argv);

	return 0;
}
