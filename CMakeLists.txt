
# @author Yuki SUGA at SSR

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")


option(BUILD_OROCHIKINEMATICS_PLUGIN "Building OrochiKinematicsPlugin" OFF)

if(NOT BUILD_OROCHIKINEMATICS_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "OrochiKinematics Plugin requires CorbaPlugin.")
  endif()
endif()

set(idls idl/TrajectoryPlanner.idl)

macro(_IDL_OUTPUTS _idl _dir _result)
    set(${_result} ${_dir}/${_idl}SK.cc ${_dir}/${_idl}.hh
        ${_dir}/${_idl}DynSK.cc)
endmacro(_IDL_OUTPUTS)

macro(_COMPILE_IDL _idl_file)
    if(NOT WIN32)
        execute_process(COMMAND rtm-config --prefix OUTPUT_VARIABLE OPENRTM_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(COMMAND rtm-config --idlflags OUTPUT_VARIABLE OPENRTM_IDLFLAGS
        OUTPUT_STRIP_TRAILING_WHITESPACE)
        separate_arguments(OPENRTM_IDLFLAGS)
        execute_process(COMMAND rtm-config --idlc OUTPUT_VARIABLE OPENRTM_IDLC
        OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(_rtm_skelwrapper_command "rtm-skelwrapper")
    else(NOT WIN32)
        set(_rtm_skelwrapper_command "rtm-skelwrapper.py")
    endif(NOT WIN32)
    get_filename_component(_idl ${_idl_file} NAME_WE)
    set(_idl_srcs_var ${_idl}_SRCS)
    _IDL_OUTPUTS(${_idl} ${CMAKE_CURRENT_BINARY_DIR} ${_idl_srcs_var})

    add_custom_command(OUTPUT ${${_idl_srcs_var}}
        COMMAND python ${OPENRTM_DIR}/bin/${_rtm_skelwrapper_command} --include-dir= --skel-suffix=Skel --stub-suffix=Stub --idl-file=${_idl}.idl 
        COMMAND ${OPENRTM_IDLC} ${OPENRTM_IDLFLAGS} ${_idl_file}
        WORKING_DIRECTORY ${CURRENT_BINARY_DIR}
        DEPENDS ${_idl_file}
        COMMENT "Compiling ${_idl_file}" VERBATIM)
    add_custom_target(${_idl}_OROCHI_TGT DEPENDS ${${_idl_srcs_var}})
    set(ALL_IDL_SRCS ${ALL_IDL_SRCS} ${${_idl_srcs_var}})
    if(NOT TARGET ALL_IDL_TGT)
        add_custom_target(ALL_IDL_TGT)
    endif(NOT TARGET ALL_IDL_TGT)
    add_dependencies(ALL_IDL_TGT ${_idl}_OROCHI_TGT)
endmacro(_COMPILE_IDL)

# Module exposed to the user
macro(OPENRTM_COMPILE_IDL_FILES)
    foreach(idl ${ARGN})
        _COMPILE_IDL(${idl})
    endforeach(idl)
endmacro(OPENRTM_COMPILE_IDL_FILES)


OPENRTM_COMPILE_IDL_FILES(${idls})
set(ALL_IDL_SRCS ${ALL_IDL_SRCS} PARENT_SCOPE)

# Module exposed to the user
macro(OPENRTM_COMPILE_IDL_FILES)
    foreach(idl ${ARGN})
        _COMPILE_IDL(${idl})
    endforeach(idl)
endmacro(OPENRTM_COMPILE_IDL_FILES)

set(IDL_INCLUDE_DIRS
  ${OPENRTM_PREFIX}/include/openrtm-1.1
  ${OPENRTM_PREFIX}/include/coil-1.1
  ${OPENRTM_PREFIX}/include/openrtm-1.1/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl
  idl )
idl_compile_cpp(idl_cpp_files idl_h_files idl TrajectoryPlanner)
include_directories(${IDL_INCLUDE_DIRS})


# Plugin
set(plugin CnoidOrochiKinematicsPlugin)

set(plugin_sources
  OrochiKinematicsPlugin.cpp
  OrochiKinematicsRTC_Cnoid.cpp
  TrajectoryPlannerSVC_impl.cpp
  ${idl_cpp_files})

set(plugin_headers
  OrochiKinematicsPlugin.h
  OrochiKinematicsRTC_Cnoid.h
  TrajectoryPlannerSVC_impl.h
  ${idl_h_files})

if(OPENRTM_VERSION STREQUAL "1.1.0")
  add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles})
else()
  add_cnoid_OpenRTM_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles})
  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    set_source_files_properties(BodyRTCItem.cpp
                                VirtualRobotRTC.cpp
                                VirtualRobotPortHandler.cpp
                                BridgeConf.cpp
                                RTCItem.cpp
                                OpenHRPClockGeneratorItem.cpp
                                ChoreonoidExecutionContext.cpp 
                                PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
  endif()
endif()


add_dependencies(${plugin} ALL_IDL_TGT)

target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidOpenRTM ${OPENRTM_LIBRARIES})
if(UNIX)
  target_link_libraries(${plugin} ${Boost_REGEX_LIBRARY})
endif()
apply_common_setting_for_plugin(${plugin} "${plugin_headers}")


# if(ENABLE_PYTHON)
#  add_subdirectory(python)
# endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc)
install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc
  FILES_MATCHING PATTERN "*" EXCLUDE)

