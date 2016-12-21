/*! @file TrajectoryRecorder.h
 *
 *  @brief
 *  See TrajectoryRecorder class.
 *
 *  @copyright 2016 DJI Research LLC. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef DJI_TRAJECTORYRECORDER_H
#define DJI_TRAJECTORYRECORDER_H

#include <DJI_API.h>
#include <DJI_Type.h>
#include <gps2local/CartesianFrame.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Core>

class TrajectoryRecorder {
 public:
  TrajectoryRecorder(){};
  TrajectoryRecorder(DJI::onboardSDK::CoreAPI* api);
  FILE * createFile();
  void record();
 private:
  DJI::onboardSDK::CoreAPI* api;
  bool continueRecording = false;
};

#endif //DJI_TRAJECTORYRECORDER_H
