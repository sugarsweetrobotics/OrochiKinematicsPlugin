// -*-C++-*-
/*!
 * @file  TrajectoryPlannerSVC_impl.h
 * @brief Service implementation header of TrajectoryPlanner.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "TrajectoryPlannerSkel.h"

class OrochiKinematicsRTC_Cnoid;

#ifndef TRAJECTORYPLANNERSVC_IMPL_H
#define TRAJECTORYPLANNERSVC_IMPL_H
 
/*!
 * @class ObjectDetectionServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ObjectDetectionService
 */
class Manipulation_ObjectDetectionServiceSVC_impl
 : public virtual POA_Manipulation::ObjectDetectionService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ObjectDetectionServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ObjectDetectionServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ObjectDetectionServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* detectObject(const Manipulation::ObjectIdentifier& objectID, Manipulation::ObjectInfo_out objInfo);
   Manipulation::ReturnValue* setBaseFrame(const Manipulation::Matrix34& frame);

};

/*!
 * @class ObjectHandleStrategyServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ObjectHandleStrategyService
 */
class Manipulation_ObjectHandleStrategyServiceSVC_impl
 : public virtual POA_Manipulation::ObjectHandleStrategyService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ObjectHandleStrategyServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ObjectHandleStrategyServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ObjectHandleStrategyServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* getApproachOrientation(const Manipulation::ObjectInfo& objInfo, Manipulation::EndEffectorPose& eePos);

};

/*!
 * @class KinematicSolverServiceSVC_impl
 * Example class implementing IDL interface Manipulation::KinematicSolverService
 */
class Manipulation_KinematicSolverServiceSVC_impl
 : public virtual POA_Manipulation::KinematicSolverService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~KinematicSolverServiceSVC_impl();

  OrochiKinematicsRTC_Cnoid* m_pRTC;
public:
  void setRTC(OrochiKinematicsRTC_Cnoid* pRTC) { m_pRTC = pRTC; }
 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_KinematicSolverServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_KinematicSolverServiceSVC_impl();

   // attributes and operations
  Manipulation::ReturnValue* solveKinematics(const Manipulation::EndEffectorPose& targetPose, const Manipulation::JointAngleSeq& startJointAngles, Manipulation::JointAngleSeq_out targetJointAngles);

};

/*!
 * @class CollisionDetectionServiceSVC_impl
 * Example class implementing IDL interface Manipulation::CollisionDetectionService
 */
class Manipulation_CollisionDetectionServiceSVC_impl
 : public virtual POA_Manipulation::CollisionDetectionService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~CollisionDetectionServiceSVC_impl();
private:

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_CollisionDetectionServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_CollisionDetectionServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* isCollide(const Manipulation::RobotIdentifier& robotID, const Manipulation::JointAngleSeq& jointAngles, Manipulation::CollisionPairSeq_out collisions);

};

/*!
 * @class ManipulationPlannerServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ManipulationPlannerService
 */
class Manipulation_ManipulationPlannerServiceSVC_impl
 : public virtual POA_Manipulation::ManipulationPlannerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ManipulationPlannerServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ManipulationPlannerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ManipulationPlannerServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* planManipulation(const Manipulation::RobotJointInfo& jointsInfo, const Manipulation::JointAngleSeq& startJointAngles, const Manipulation::JointAngleSeq& goalJointAngles, Manipulation::ManipulationPlan_out manipPlan);

};

/*!
 * @class ModelServerServiceSVC_impl
 * Example class implementing IDL interface Manipulation::ModelServerService
 */
class Manipulation_ModelServerServiceSVC_impl
 : public virtual POA_Manipulation::ModelServerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~ModelServerServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_ModelServerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_ModelServerServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* getModelInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::RobotJointInfo_out jointsInfo);
   Manipulation::ReturnValue* getMeshInfo(const Manipulation::RobotIdentifier& robotID, Manipulation::MeshInfo_out mesh);

};

/*!
 * @class MotionGeneratorServiceSVC_impl
 * Example class implementing IDL interface Manipulation::MotionGeneratorService
 */
class Manipulation_MotionGeneratorServiceSVC_impl
 : public virtual POA_Manipulation::MotionGeneratorService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~MotionGeneratorServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
  Manipulation_MotionGeneratorServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~Manipulation_MotionGeneratorServiceSVC_impl();

   // attributes and operations
   Manipulation::ReturnValue* followManipPlan(const Manipulation::ManipulationPlan& manipPlan);
   Manipulation::ReturnValue* getCurrentRobotJointAngles(Manipulation::JointAngleSeq_out jointAngles);

};



#endif // TRAJECTORYPLANNERSVC_IMPL_H


