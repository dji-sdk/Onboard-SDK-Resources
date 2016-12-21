//
// Created by rohit on 12/13/16.
//

#ifndef ONBOARDSDK_INTERNAL_SIMPLETRAJECTORY_H
#define ONBOARDSDK_INTERNAL_SIMPLETRAJECTORY_H

#include <trajectory-follower/Trajectory.h>

class SimpleTrajectory: public Trajectory {
 public:
  void generate();
 private:
  double _end_time;
};
#endif //ONBOARDSDK_INTERNAL_SIMPLETRAJECTORY_H
