/*! @file TrajectoryInfrastructure.h
 *
 *  @brief
 *  See TrajectoryInfrastructure class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H
#define ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H

//! Declare helper functions

#include <se3controller/SE3Controller.h>
#include <OnboardSDK.h>
#include <trajectory-follower/Trajectory.h>
#include <trajectory-follower/TrajectoryFollower.h>
#include <trajectory-recorder/TrajectoryRecorder.h>
#include <trajectory-generator/ParametricSpiral.h>
#include <trajectory-ros-interface/TrajectoryTCPInterface.h>

#ifndef C_PI
#define C_PI (double) 3.141592653589793
#endif

//! Wrappers for all parts of the trajectory functionality.
//! These are the functions you will call from your programs linking to this library
namespace TrajectoryInfrastructure {

  //! Handle state broadcast data from OSDK
  int startStateBroadcast(DJI::onboardSDK::CoreAPI *api);

  //! Trajectory Recorder state broadcast settings. Not supported in the current release
  int startRecordBroadcast(DJI::onboardSDK::CoreAPI *api);

  //! This function takes among other things a csv file as an argument and executes that mission
  void executeFromCsv(CoreAPI *api,
                      Flight *flight,
                      Camera *camera,
                      char *trajFile,
                      double timescale,
                      std::string droneVer,
                      std::string controllerParamFile);

  //! This function takes pointers to various pre-initialized objects
  uint16_t executeFromParams(CoreAPI *api,
                             CartesianFrame *localFrame,
                             Eigen::Vector3d originLLA,
                             Trajectory *spiral,
                             TrajectoryFollower *follower);

  //! This function sets up the follower
  TrajectoryFollower* setupFollower(CoreAPI *api,
                                    Flight *flight,
                                    CartesianFrame *localFrame,
                                    Camera *camera,
                                    std::string droneVer,
                                    std::string controllerParamFile);

  //! Create a trajectory object and check the trajectory for feasibility
  Trajectory* setupTrajectory(std::string trajParamsFile);

  //! Farm out the recorder to a different thread
  void *recordInThread(void *recorderPtr);
  //! Farm out the trajectory interface to a different thread
  void *rosInterfaceInThread(void *followerPtr);
}



#endif //ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H
