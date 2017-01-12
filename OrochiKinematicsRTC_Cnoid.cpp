// -*- C++ -*-
/*!
 * @file  OrochiKinematicsRTC_Cnoid.cpp
 * @brief OrochiKinematicsRTC Service on Choreonoid
 * @date $Date$
 *
 * $Id$
 */

#include "OrochiKinematicsRTC_Cnoid.h"

// Module specification
// <rtc-template block="module_spec">
static const char* orochikinematicsrtc_cnoid_spec[] =
  {
    "implementation_id", "OrochiKinematicsRTC_Cnoid",
    "type_name",         "OrochiKinematicsRTC_Cnoid",
    "description",       "OrochiKinematicsRTC Service on Choreonoid",
    "version",           "1.0.0",
    "vendor",            "Ogata Laboratory",
    "category",          "MotionPlanning",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",

    // Widget
    "conf.__widget__.debug", "text",
    // Constraints

    "conf.__type__.debug", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
OrochiKinematicsRTC_Cnoid::OrochiKinematicsRTC_Cnoid(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_kinematicsPort("kinematics")


    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
OrochiKinematicsRTC_Cnoid::~OrochiKinematicsRTC_Cnoid()
{
}



RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports

  m_kinematicsPort.registerProvider("Manipulation_KinematicSolverService",
				    "Manipulation::KinematicSolverService",
				    m_kinematicsService);

  // Set service consumers to Ports
  
  // Set CORBA Service Ports
   addPort(m_kinematicsPort);
  ///m_kinematicsService.setRTC(this);
   m_kinematicsService.setRTC(this);  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t OrochiKinematicsRTC_Cnoid::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void OrochiKinematicsRTC_CnoidInit(RTC::Manager* manager)
  {
    coil::Properties profile(orochikinematicsrtc_cnoid_spec);
    manager->registerFactory(profile,
                             RTC::Create<OrochiKinematicsRTC_Cnoid>,
                             RTC::Delete<OrochiKinematicsRTC_Cnoid>);
  }
  
};


