/*! @file TrajectoryFollower.h
 *
 *  @brief
 *  See TrajectoryFollower class.
 *
 *  @copyright 2016 DJI Research LLC. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef DJI_FOLLOWTRAJECTORY_H
#define DJI_FOLLOWTRAJECTORY_H

#include "OnboardSDK.h"
#include "gps2local/CartesianFrame.h"
#include "Trajectory.h"
#include "se3controller/SE3Controller.h"

//! @brief Function for following a trajectory. Uses CoreAPI, Flight, LocalFrame and Curve classes.
//! @details This function iterates over a sampled trajectory (assumed to be feasible) and calls
//! a position controller to reach each desired setpoint.
class TrajectoryFollower{
 public:
  int followTrajectory(DJI::onboardSDK::CoreAPI *osdkApi,
                         DJI::onboardSDK::Flight *flight,
                         CartesianFrame *localFrame,
                         Trajectory *trajectory,
                         DJI::onboardSDK::Camera *camera,
                         std::string droneVer);

};



#endif //DJI_FOLLOWTRAJECTORY_H
