/*! @file SimpleTrajectory.h
 *
 *  @brief
 *  Step-input trajectory. Unavailable in v1.0.2 Precision Missions/3.2 Onboard SDK
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef DJI_SIMPLETRAJECTORY_H
#define DJI_SIMPLETRAJECTORY_H

#include <trajectory-follower/Trajectory.h>

class SimpleTrajectory: public Trajectory {
 public:
  void generate();
 private:
  double _end_time;
};
#endif //ONBOARDSDK_INTERNAL_SIMPLETRAJECTORY_H
