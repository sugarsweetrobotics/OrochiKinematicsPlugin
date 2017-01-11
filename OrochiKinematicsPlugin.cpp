/**
 *   @author Yuki Suga at SSR
 */

#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ToolBar>
#include <cnoid/OpenRTMUtil>

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
  //pRTC = dynamic_cast<OrochiKinematicsRTC_Cnoid*>(rtc);
  //pRTC->setPlugin(this);

  return true;
}


void OrochiKinematicsPlugin::onTest() {
}


Return_t OrochiKinematicsPlugin::inverseKinematics(const ::Pose3D& eePose, const std::vector<double> startJointAngles, std::vector<double>& resultJointAngles) {
  Return_t retval;

  std::string name = "orochi";
  cnoid::BodyItemPtr targetBodyItem;
  
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
  // Link* wrist = body->link("j6");
  
  return retval;
}

//////
CNOID_IMPLEMENT_PLUGIN_ENTRY(OrochiKinematicsPlugin)
