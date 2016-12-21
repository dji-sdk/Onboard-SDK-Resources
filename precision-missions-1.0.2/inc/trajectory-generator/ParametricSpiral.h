/*! @file ParamtericSpiral.h
 *
 *  @brief
 *  See ParamtericSpiral class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef ONBOARDSDK_PARAMETRICSPIRAL_H
#define ONBOARDSDK_PARAMETRICSPIRAL_H

#include <trajectory-follower/Trajectory.h>
#include <cmath>
#include <fstream>
#include <rapidjson/document.h>

/*!@brief: This class has methods for generating a geolocated, analytical spiral
 * given a set of parameters.
 * @details: Also has methods for checking feasibility of the planned spiral and for reading json files
 */
//! As a user, you do not have to interface with these functions directly.
class ParametricSpiral : public Trajectory{
 public:
  ParametricSpiral(double start_radius, double end_radius, double start_angle, int rotations, double horizontal_speed, double vertical_speed);
  ParametricSpiral(const std::string &spiralParamFile);
  void generate();
  bool checkFeasibility();
  void readJson(std::string paramFile);

 private:
  void generateEndTime();
  void validateHeights();
  //! Position queries
  double xFunc(double theta);
  double yFunc(double theta);
  double radius(double theta);
  double zFunc(double theta);

  //! Velocity queries
  double xVelFunc(double theta, double thetaPrime);
  double yVelFunc(double theta, double thetaPrime);
  double zVelFunc(double thetaPrime);

  //! Acceleration queries
  double xAccFunc(double theta, double thetaPrime, double thetaDoublePrime);
  double yAccFunc(double theta, double thetaPrime, double thetaDoublePrime);
  //zAcc = 0 at all times

  //! Independent variables
  double tFunc(double theta);
  double thetaFunc(double t);
  double thetaPrimeFunc(double t);
  double thetaDoublePrimeFunc(double t);

  std::string _spiralParamFile;
  double _start_radius;
  double _end_radius;
  double _start_angle;
  int _rotations;
  double _horizontal_speed;
  double _pitch;
  double _end_time;
  double _start_agl;

  //! Temp parameters
  double _xOffset;
  double _yOffset;
  //! This is used to pick between MSL and AGL
  double _zOffset;

  //! GPs handling
  double _gps_lat;
  double _gps_lon;
  double _gps_msl;

  bool _isMslValid;
  bool _isAglValid;
};

#endif //ONBOARDSDK_PARAMETRICSPIRAL_H
