/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ToolBar>
#include <cnoid/OpenRTMUtil>
#include <cnoid/LazyCaller>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <boost/bind.hpp>

#include "OrochiKinematicsPlugin.h"

#include "OrochiKinematicsRTC_Cnoid.h"

/**
 * Constructor
 */
OrochiKinematicsPlugin::OrochiKinematicsPlugin() : cnoid::Plugin("OrochiKinematics") {
  require("Body");
  require("OpenRTM");
}


/**
 */
bool OrochiKinematicsPlugin::initialize() {
  cnoid::ToolBar* bar = new cnoid::ToolBar("OrochiKinematics");
  bar->addButton("Test")->sigClicked().connect(boost::bind(&OrochiKinematicsPlugin::onTest, this));
  addToolBar(bar);

  RTC::Manager& rtcManager = RTC::Manager::instance();
  OrochiKinematicsRTC_CnoidInit(&rtcManager);
  const char* param = "OrochiKinematicsRTC_Cnoid?instance_name=OrochiKinematicsRTC_Cnoid&exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30";
  RTObject_impl* rtc = rtcManager.createComponent(param);
  pRTC = dynamic_cast<OrochiKinematicsRTC_Cnoid*>(rtc);
  pRTC->setPlugin(this);

  return true;
}


void OrochiKinematicsPlugin::onTest() {
  ::Pose3D eePose;
  eePose.position.x = 0.0880713;
  eePose.position.y = -0.00525294;
  eePose.position.z = 0.740525;
  eePose.orientation.r = 0.140208;
  eePose.orientation.p = 0.44425;
  eePose.orientation.y = 0.205369;

  eePose.position.z -= 0.05;
  
  
  std::vector<double> startJointAngles;
  startJointAngles.push_back(-0.2618);
  startJointAngles.push_back(-0.2094);
  startJointAngles.push_back(1.0472);
  startJointAngles.push_back(0.6981);
  startJointAngles.push_back(0.6981);
  startJointAngles.push_back(-0.5236);
  //startJointAngles.push_back(0.0);
  std::vector<double> targetJointAngles;

  

  Return_t retval = this->inverseKinematics(eePose, startJointAngles, targetJointAngles);
  std::cout << "retval = " << retval.message << std::endl;
  
}

void OrochiKinematicsPlugin::onKinematicStateChanged(const std::string& name) {
  std::cout << "onKinematcStateChanged: " << name << std::endl;
  namedCounter[name] = namedCounter[name] + 1;
}

Return_t OrochiKinematicsPlugin::inverseKinematicsSynchronously(const ::Pose3D& eePose, const std::vector<double> startJointAngles, std::vector<double>& resultJointAngles) {
  __eePose = eePose;
  __startJointAngles = startJointAngles;
  __resultJointAngles.clear();
  cnoid::callSynchronously(boost::bind(&OrochiKinematicsPlugin::__inverseKinematics, this));
  resultJointAngles.insert(resultJointAngles.end(), __resultJointAngles.begin(), __resultJointAngles.end());
  return __retval;
}

Return_t OrochiKinematicsPlugin::inverseKinematics(const ::Pose3D& eePose, const std::vector<double> startJointAngles, std::vector<double>& resultJointAngles) {
  Return_t retval;

  std::cout << "OrochiKinematicsPlugin::inverseKinematics(\n";
  std::cout << "  eePose::Pose3D(" << eePose.position.x << ", " << eePose.position.y << ", " << eePose.position.z << ", \n";
  std::cout << "                 " << eePose.orientation.r << ", " << eePose.orientation.y << ", " << eePose.orientation.p << "),\n";
  std::cout << "  startAngles::vector<double>(" << startJointAngles[0] << ", " << startJointAngles[1] << ", " << startJointAngles[2] << ", \n";
  std::cout << "                              " << startJointAngles[3] << ", " << startJointAngles[4] << ", " << startJointAngles[5] << "))\n";

  
  std::string name = "orochi";
  cnoid::BodyItemPtr targetBodyItem;

  if (startJointAngles.size() != 6) {
    retval.returnValue = RETVAL_INVALID_JOINT_NUM;
    retval.message = "ERROR.OrochiKinematics needs 6 jointSeq values.";
    return retval;
  }
  
  cnoid::ItemList<cnoid::BodyItem> bodyItems = cnoid::ItemTreeView::instance()->checkedItems<cnoid::BodyItem>();
  for(size_t i = 0;i < bodyItems.size(); ++i) {
    cnoid::BodyPtr body = bodyItems[i]->body();
    if (body->name() == name) {
      targetBodyItem = bodyItems[i];
    }
  }
  
  if (!targetBodyItem) {
    retval.returnValue = RETVAL_MODEL_NOT_FOUND;
    retval.message = "Model is not found.";
    return retval;
  }
  cnoid::BodyPtr body = targetBodyItem->body();

  /**
  cnoid::WorldItemPtr world = targetBodyItem->findOwnerItem<cnoid::WorldItem>();
  if (!world) {
    retval.returnValue = RETVAL_INVALID_PRECONDITION;
    retval.message = "Choreonoid needs WorldItem to Kinematics Calculation";
    return retval;
  }
  */
  
  std::map<std::string, int32_t>::iterator itr = namedCounter.find(name);
  if (itr == namedCounter.end()) {
    namedCounter[name] = 0;
    //world->sigCollisionsUpdated()
    targetBodyItem->sigKinematicStateChanged()    
      .connect(boost::bind(&OrochiKinematicsPlugin::onKinematicStateChanged, this, name));
  }


  for(int j=0; j < startJointAngles.size(); ++j){
    body->joint(j)->q() = startJointAngles[j];
  }
  targetBodyItem->notifyKinematicStateChange(true); 

  /*
  while(true) {
    if (namedCounter[name] > 0) {
      break;
    }
    ; // do nothing
  }
  */
  cnoid::MessageView::instance()->flush();

  
  cnoid::LinkPtr base = body->rootLink();
  cnoid::LinkPtr wrist = body->link("j6");
  cnoid::JointPathPtr baseToWrist = cnoid::getCustomJointPath(body, base, wrist);
  cnoid::Vector3 currentXYZ = wrist->p();
  cnoid::Vector3 currentRPY = cnoid::rpyFromRot(wrist->attitude());
  std::cout << " - CurrentPose3D:" << std::endl;
  std::cout << " " << currentXYZ[0] << ", " << currentXYZ[1] << ", " << currentXYZ[2] << "\n";
  std::cout << " " << currentRPY[0] << ", " << currentRPY[1] << ", " << currentRPY[2] << "\n";  
  
  
  cnoid::Vector3 rpy;
  rpy[0] = eePose.orientation.r;
  rpy[1] = eePose.orientation.p;
  rpy[2] = eePose.orientation.y;
  cnoid::Vector3 xyz;
  xyz[0] = eePose.position.x;
  xyz[1] = eePose.position.y;
  xyz[2] = eePose.position.z;

  if(!baseToWrist->calcInverseKinematics(xyz, (cnoid::rotFromRpy(rpy)))) {
      retval.returnValue = RETVAL_INVALID_ARGUMENT;
      retval.message = "Cannot Solve Inverse Kinematics";
      return retval;
  } else {
    for(int i = 0;i < startJointAngles.size();i++) {
      resultJointAngles.push_back(baseToWrist->joint(i)->q());
      body->joint(i)->q() = baseToWrist->joint(i)->q();
    }
  }
  targetBodyItem->notifyKinematicStateChange(true); 
  
  retval.returnValue = RETVAL_OK;
  retval.message = "OK";
  return retval;
}

//////
CNOID_IMPLEMENT_PLUGIN_ENTRY(OrochiKinematicsPlugin)
