//
// Created by rohit on 11/10/16.
//

#ifndef ONBOARDSDK_INTERNAL_TRAJECTORYROSINTERFACE_H
#define ONBOARDSDK_INTERNAL_TRAJECTORYROSINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <sstream>
#include <iostream>
#include <pthread.h>
#include <fcntl.h>
#include <DJI_App.h>
#include <DJI_API.h>
#include <trajectory-follower/TrajectoryFollower.h>

//! Struct size = 80 bytes
#pragma pack (1)
typedef struct {
  //! Velocity
  double vx;
  double vy;
  double vz;
  //! Velocity Health - can we trust the velocity?
  uint32_t health_flag;
  //! Attitude
  double q0;
  double q1;
  double q2;
  double q3;
  //! Command velocity
  double ux;
  double uy;
  double uz;
} stateData;
#pragma pack()


//! Adapted from StackOverflow: Thread helper class to get around PTMF incompatibility with void *, pthread's preferred function pointer type
class ThreadHelper
{
 public:
  ThreadHelper() {/* empty */}
  virtual ~ThreadHelper() {/* empty */}

  /** Returns true if the thread was successfully started, false if there was an error starting the thread */
  bool StartInternalThread()
  {
    return (pthread_create(&_thread, NULL, threadWrapperFunc, this) == 0);
  }

  /** Will not return until the internal thread has exited. */
  void WaitForInternalThreadToExit()
  {
    (void) pthread_join(_thread, NULL);
  }

 protected:
  /** Implement this method in your subclass with the code you want your thread to run. */
  virtual void threadWrapper() = 0;
  int sockfd;

 private:
  static void * threadWrapperFunc(void *This) { ((ThreadHelper *) This)->threadWrapper(); return NULL;}

  pthread_t _thread;
};


class TrajectoryROSInterface: public ThreadHelper {
 public:
  TrajectoryROSInterface(int port){this->port = port; pauseLock = PTHREAD_MUTEX_INITIALIZER;}

  void setTrajectoryPointer(TrajectoryFollower* trajectoryPtr) {this->trajectoryPtr = trajectoryPtr;}
  void setApiPtr(DJI::onboardSDK::CoreAPI* osdkApi) {this->api = osdkApi;}

  int publish();
  void threadWrapper();

 private:
  int publisherRead(int sockfd);
  void lockPause () {pthread_mutex_lock(&this->pauseLock);}
  void unlockPause () {pthread_mutex_unlock(&this->pauseLock);}
  void error(const char *msg);

  int port;
  bool pause;
  pthread_mutex_t pauseLock;
  DJI::onboardSDK::CoreAPI* api;
  TrajectoryFollower* trajectoryPtr;
};

#endif //ONBOARDSDK_INTERNAL_TRAJECTORYROSINTERFACE_H
