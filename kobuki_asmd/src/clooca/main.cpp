#include <sys/time.h>
#include <signal.h>

#include "main.h"
#include "stm.h"
#include "stmEventManager.hpp"

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

  std::cout << "Demo : Example of simple control loop." << std::endl;
  //KobukiManager kobuki_manager;

  ecl::Sleep sleep(0.1);
  ecl::Pose2D<double> pose;
  try {
    while (!shutdown_req){
      event.checkEvent( stm );
      eventManager.execute();
      sleep();
      pose = stm.kobukimanager->getPose();
      //std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }

	return 0;
}
