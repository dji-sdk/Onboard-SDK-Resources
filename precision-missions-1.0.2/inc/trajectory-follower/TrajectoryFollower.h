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
#include "Eigen/Core"
#include <fstream>
#include <rapidjson/document.h>


//! @brief Function for following a trajectory. Uses CoreAPI, Flight, LocalFrame and Curve classes.
//! @details This function iterates over a sampled trajectory (assumed to be feasible) and calls
//! a position controller to reach each desired setpoint.
class TrajectoryFollower{
 public:
  //! Constructor
  TrajectoryFollower(DJI::onboardSDK::CoreAPI *osdkApi, DJI::onboardSDK::Flight *flight,
                       DJI::onboardSDK::Camera *camera, CartesianFrame *localFrame, std::string droneVer, std::string tuningParamFile);

  //! Set the controller paramerters from json file
  void readControllerParams(const std::string &tuningParamFile);
  int setControllerParams(SE3Controller* posCtl);

  //! Main following function
  int followTrajectory(Trajectory *trajectory);

  bool holdPosition(Eigen::Vector3d holdPos, double holdYaw, DJI::onboardSDK::CoreAPI *osdkApi,
                      DJI::onboardSDK::Flight *flight, CartesianFrame *localFrame, SE3Controller *posCtl, double timestep);

  //! PauseResumeHandling
  void pauseResumeHandling(Trajectory *trajectory, double cur_time);

  //! Interact with the pause boolean
  bool getPause();
  bool setPause(bool pauseVal);

  //! Interact with the readyToResume boolean
  bool getReadyToResume();
  bool setReadyToResume(bool resumeVal);

  //! Interact with the internal variables
  void setApi(DJI::onboardSDK::CoreAPI *osdkApi) {this->api = osdkApi;}
  DJI::onboardSDK::CoreAPI *getApi() { return api;}

  void setFlight(DJI::onboardSDK::Flight *flight) {this->flight = flight;}
  DJI::onboardSDK::Flight *getFlight() { return flight;}

  void setCamera(DJI::onboardSDK::Camera *camera) {this->camera = camera;}
  DJI::onboardSDK::Camera *getCamera() { return camera;}

  void setLocalFrame(CartesianFrame *localFrame) {this->localFrame = localFrame;}
  CartesianFrame *getLocalFrame() { return localFrame;}

  void setDrone(std::string droneVer) {this->droneVer = droneVer;}
  std::string getDrone() { return droneVer;}

  Eigen::Vector3d getCommand() {return this->command; }
 private:
  //! DJI Variables
  DJI::onboardSDK::CoreAPI *api;
  DJI::onboardSDK::Flight *flight;
  DJI::onboardSDK::Camera *camera;
  CartesianFrame *localFrame;
  std::string droneVer;

  //! Function to read json file
  void readJson(std::string paramFile);

  //! Current command - useful for signaling intent. Current uses: Trajectory-Collision Avoidance interface
  Eigen::Vector3d command;

  //! Pause/Resume helpers
  void lockPause () {pthread_mutex_lock(&this->pauseLock);}
  void unlockPause () {pthread_mutex_unlock(&this->pauseLock);}
  void lockResume () {pthread_mutex_lock(&this->resumeLock);}
  void unlockResume () {pthread_mutex_unlock(&this->resumeLock);}

  //! State machine:
  //! (1) Moving
  //! (2) Pausing
  //! (3) Paused
  //! (4) Resuming

  //! State variables to help monitor state
  bool pause;
  bool pauseInProgress;
  bool readyToResume;
  bool handledResume;
  bool wasPaused;

  //! Pthread variable
  pthread_mutex_t pauseLock;
  pthread_mutex_t resumeLock;

  //! Controller tuning variables
  float P_x, P_y, P_z, D_x, D_y, D_z, I_x, I_y, I_z;
  float mass;
  float thrust_scale;
  bool paramsFromFile;
};

#endif //DJI_FOLLOWTRAJECTORY_H

