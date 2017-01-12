#pragma once

/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/IdPair>
#include <cnoid/Plugin>

#include <cnoid/CollisionDetector>

#include <map>
/// Reference class 
class OrochiKinematicsRTC_Cnoid;

enum RETVAL {
  RETVAL_OK,
  RETVAL_MODEL_NOT_FOUND,
  RETVAL_INVALID_PRECONDITION,
  RETVAL_INVALID_JOINT_NUM,
  RETVAL_INVALID_ARGUMENT,
};

struct Return_t {
Return_t() : returnValue(RETVAL_OK), message("OK") {};
Return_t(RETVAL returnValue_, const std::string& message_): returnValue(returnValue_), message(message_) {};
  
  RETVAL returnValue;
  std::string message;
};


struct Point3D {
  double x, y, z;
};

struct Orientation3D {
  double r, p, y;
};

struct Pose3D {
  Point3D position;
  Orientation3D orientation;
};

/**
 * Plugin Class
 */
class OrochiKinematicsPlugin : public cnoid::Plugin {
private:
  OrochiKinematicsRTC_Cnoid* pRTC;

public:

  
  /**
   * Constructor
   */
  OrochiKinematicsPlugin();

public:
  /**
   * Initialization Function called by System
   */
  virtual bool initialize();

  void onTest();

  std::map<std::string, int32_t> namedCounter;
  void onKinematicStateChanged(const std::string& name);
  
  Return_t inverseKinematics(const Pose3D& eePose, const std::vector<double> startJointAngles, std::vector<double>& resultJointAngles);
};


