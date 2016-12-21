/*! @file Trajectory.h
 *
 *  @brief
 *  See Trajectory class.
 *
 *  @copyright 2016 DJI Research LLC. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef DJI_TRAJECTORY_H
#define DJI_TRAJECTORY_H

#include <string>
#include <vector>
#include <twopoint-trajectory-generator/FeasibleTrajectory.h>
#include <gps2local/CartesianFrame.h>
#include <Eigen/Core>
#include <csv.h>
#include <twopoint-trajectory-generator/R3Trajectory.h>
#include <iostream>

#define C_PI (double) 3.141592653589793

typedef struct {
  double tau;
  double x;
  double y;
  double z;
  double yaw;
} CurvePtPos;

typedef struct {
  double tau;
  double vx;
  double vy;
  double vz;
} CurvePtVel;

typedef struct {
  double tau;
  double ax;
  double ay;
  double az;
} CurvePtAcc;

//! @brief The Trajectory class is the core of the DJI Onboard SDK trajectory follower
class Trajectory {
 public:
  //! initialize the trajectory from file.
  Trajectory(std::string curve_path);
  Trajectory(){};

  //! Read CSV file containing info about curve. This low-level interface should be avoided.
  virtual void readCsv(std::string curve_path);

  //! Read Json - virtual function that needs implementation in the trajectory of choice.
  virtual void readJson(std::string paramFile);

  //! Generate the trajectory from the user's parameter file
  virtual void generate();
  // Check if GPS coordinates for the curve exist in the file
  void handleGPS(char *gpsLine);

  // Change the coordinates of the curve if you need to move to GPS
  void geolocate(Eigen::Vector3d originLLA);

  // retrieve a point a given time.
  // return: 
  //    CurvePtPos (x,y,z,yaw) at time t.
  //    yaw is guaranteed to be between [-\pi, pi).
  CurvePtPos pos_at(double t);

  // retrieve (numerical derivative) velocity at a given time.
  CurvePtVel vel_at(double t);

  // retrieve (numerical derivative) acceleration at time t.
  CurvePtAcc acc_at(double t);

  // Timescale
  void timescale(double speedfactor);

  //Get to start point
  void createEntryTrajectory(CurvePtPos curPos);

  // Insert resume trajectory into current trajectory
  bool rescaleResume(double curTime, CurvePtPos curPos);

  //Getter functions
  CurvePtPos getStartPt() { return start_pt; }
  double getStartTime() { return time_start; }
  double getEndTime() { return time_end; }
  bool getTakePictures() {return takePictures; }
  int getPictureIntervalSec() {return pictureIntervalSec; }
  bool getTakeVideo() {return takeVideo; }
  double getRampUpTime() { return rampUpTime; }

  //Setter functions
  void enableLidarMapping(bool enableMap) { lidarMappingBool = enableMap; }
  void enableCollisionAvoidance(bool enableCA) { collisionAvoidanceBool = enableCA; }

  //Getter functions
  bool getLidarMapping() { return lidarMappingBool; }
  bool getCollisionAvoidance() { return collisionAvoidanceBool; }

  //Print GPS location of the desired curve
  void printGPS();

 protected:
  //! Vectors for sampled trajectory, velocity and accel
  std::vector<CurvePtPos> path;
  std::vector<CurvePtVel> pathVel;
  std::vector<CurvePtAcc> pathAcc;

  //! CSV Interface - Checks to see if these values have been set
  bool csvHasDerivatives;

  //! Picture taking
  bool takePictures;
  double pictureIntervalSec;

  //! Video capture
  bool takeVideo;

  //! LiDAR mapping
  bool lidarMappingBool;
  //! LiDAR collision avoidance
  bool collisionAvoidanceBool;

  //! Time bookkeeping
  double time_start, time_end;
  int time_i;
  double rampUpTime;

  //! Position bookkeeping
  CurvePtPos start_pt;

  //! Member variables for geolocation
  double latTraj;
  double lonTraj;
  double altTraj;
  bool gpsSet;
};

#endif //DJI_TRAJECTORY_H
