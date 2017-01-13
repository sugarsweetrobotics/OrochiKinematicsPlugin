// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.cpp
 * @brief Service implementation code of TrajectoryPlanner.idl
 *
 */


#include <iostream>
#include "TrajectoryPlannerSVC_impl.h"

#include "OrochiKinematicsPlugin.h"
#include "OrochiKinematicsRTC_Cnoid.h"

/*
 * Example implementational code for IDL interface Manipulation::ObjectDetectionService

Manipulation_ObjectDetectionServiceSVC_impl::Manipulation_ObjectDetectionServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ObjectDetectionServiceSVC_impl::~Manipulation_ObjectDetectionServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/

/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_ObjectDetectionServiceSVC_impl::detectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo)
{
	Manipulation::ReturnValue* result;
  return result;
}

Manipulation::ReturnValue* Manipulation_ObjectDetectionServiceSVC_impl::setBaseFrame(const Manipulation::Matrix34& frame)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
  return result;
}
*/


// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ObjectHandleStrategyService

Manipulation_ObjectHandleStrategyServiceSVC_impl::Manipulation_ObjectHandleStrategyServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ObjectHandleStrategyServiceSVC_impl::~Manipulation_ObjectHandleStrategyServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/

/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_ObjectHandleStrategyServiceSVC_impl::getApproachOrientation(const Manipulation::ObjectInfo& objInfo, Manipulation::EndEffectorPose& eePos)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
  return result;
}
*/


// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::KinematicSolverService
 */
Manipulation_KinematicSolverServiceSVC_impl::Manipulation_KinematicSolverServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_KinematicSolverServiceSVC_impl::~Manipulation_KinematicSolverServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
Manipulation::ReturnValue* Manipulation_KinematicSolverServiceSVC_impl::solveKinematics(const Manipulation::EndEffectorPose& targetPose, const Manipulation::JointAngleSeq& startJointAngles,  Manipulation::JointAngleSeq_out targetJointAngles)
{
  Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
  std::cout << "OrochiKinematicsRTC::solveKienmatics called." << std::endl;
  // Please insert your code here and remove the following warning pragma
  ::Pose3D eePose;
  eePose.position.x = targetPose.pose.position.x;
  eePose.position.y = targetPose.pose.position.y;
  eePose.position.z = targetPose.pose.position.z;
  eePose.orientation.r = targetPose.pose.orientation.r;
  eePose.orientation.p = targetPose.pose.orientation.p;
  eePose.orientation.y = targetPose.pose.orientation.y;
  
  std::vector<double> startJoints;
  for(int i = 0;i < startJointAngles.length();i++) {
    startJoints.push_back((double)startJointAngles[i].data);
  };
  std::vector<double> resultJoints;
  Return_t retval = m_pRTC->getPlugin()->inverseKinematics(eePose, startJoints, resultJoints);

  Manipulation::JointAngleSeq_var targetJointAngles_out(new Manipulation::JointAngleSeq());
  switch(retval.returnValue) {
  case RETVAL_OK:
    result->id = Manipulation::OK;
    targetJointAngles_out->length(resultJoints.size());
    for(int i = 0;i < resultJoints.size();i++) {
      targetJointAngles_out[i].data = resultJoints[i];
    }
    break;
  case RETVAL_MODEL_NOT_FOUND:
    result->id = Manipulation::MODEL_NOT_FOUND;
    break;
  case RETVAL_INVALID_ARGUMENT:
    result->id = Manipulation::INVALID_ARGUMENT;
    break;
  case RETVAL_INVALID_PRECONDITION:
    result->id = Manipulation::INVALID_SETTING;
    break;
  default:
    result->id = Manipulation::ERROR_UNKNOWN;
  }
  result->message = CORBA::string_dup(retval.message.c_str());
  targetJointAngles = targetJointAngles_out._retn();
  return result._retn();
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::CollisionDetectionService

Manipulation_CollisionDetectionServiceSVC_impl::Manipulation_CollisionDetectionServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_CollisionDetectionServiceSVC_impl::~Manipulation_CollisionDetectionServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/

/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_CollisionDetectionServiceSVC_impl::isCollide(const Manipulation::RobotIdentifier& robotID, const Manipulation::JointAngleSeq& jointAngles, Manipulation::CollisionPairSeq_out collisions)
{
  Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
  return result._retn();
}

*/

// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ManipulationPlannerService

Manipulation_ManipulationPlannerServiceSVC_impl::Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ManipulationPlannerServiceSVC_impl::~Manipulation_ManipulationPlannerServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/

/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_ManipulationPlannerServiceSVC_impl::planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan)>"
#endif
  return result;
}

*/

// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::ModelServerService

Manipulation_ModelServerServiceSVC_impl::Manipulation_ModelServerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_ModelServerServiceSVC_impl::~Manipulation_ModelServerServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/


/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out jointsInfo)
{
  Manipulation::ReturnValue_var result(new Manipulation::ReturnValue());
  return result._retn();
}

Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_ModelServerServiceSVC_impl::getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh)>"
#endif
  return result;
}
*/


// End of example implementational code

/*
 * Example implementational code for IDL interface Manipulation::MotionGeneratorService

Manipulation_MotionGeneratorServiceSVC_impl::Manipulation_MotionGeneratorServiceSVC_impl()
{
  // Please add extra constructor code here.
}


Manipulation_MotionGeneratorServiceSVC_impl::~Manipulation_MotionGeneratorServiceSVC_impl()
{
  // Please add extra destructor code here.
}
*/

/*
 * Methods corresponding to IDL attributes and operations

Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::followManipPlan(const Manipulation::ManipulationPlan& manipPlan)>"
#endif
  return result;
}

Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles)
{
	Manipulation::ReturnValue* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <Manipulation::ReturnValue* Manipulation_MotionGeneratorServiceSVC_impl::getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles)>"
#endif
  return result;
}
*/


// End of example implementational code



