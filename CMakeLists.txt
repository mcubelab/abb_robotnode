cmake_minimum_required(VERSION 2.8.3)
project(abb_robotnode)

add_compile_options(-std=c++14 -Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS src/egm/egm.proto)

#add_message_files(DIRECTORY msg FILES
#  robot_CartesianLog.msg
#  robot_ForceLog.msg
#  robot_JointsLog.msg
#)

# All services should go here
add_service_files(DIRECTORY srv FILES
  # Info services
  Info/Service_GetCartesian.srv
  Info/Service_GetEGMMode.srv
  Info/Service_GetFK.srv
  Info/Service_GetGripperPosition.srv
  Info/Service_GetGripperStatus.srv
  Info/Service_GetIK.srv
  Info/Service_GetJoints.srv
  Info/Service_GetRobotAngle.srv
  Info/Service_Ping.srv
  Info/Service_SetDefaults.srv
  Info/Service_SetInertia.srv
  Info/Service_SetIOSignal.srv
  Info/Service_SetMaxSpeed.srv
  Info/Service_SetTool.srv
  Info/Service_SetWorkObject.srv
  Info/Service_SetZone.srv

  # Motion services
  Motion/Service_AddToCartesianBuffer.srv
  Motion/Service_ClearCartesianBuffer.srv
  Motion/Service_ExecuteCartesianBuffer.srv
  Motion/Service_ExecuteSynchroCartesianBuffer.srv
  Motion/Service_GetCartesianBufferSize.srv
  Motion/Service_AddToJointBuffer.srv
  Motion/Service_ClearJointBuffer.srv
  Motion/Service_ExecuteJointBuffer.srv
  Motion/Service_ExecuteSynchroJointBuffer.srv
  Motion/Service_GetJointBufferSize.srv
  Motion/Service_SetCartesian.srv
  Motion/Service_SetEGMMode.srv
  Motion/Service_SetJoints.srv
  Motion/Service_SetMaxAcceleration.srv
  Motion/Service_SetMotionSupervision.srv
  Motion/Service_Stop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# All services should go here
add_executable(abb_robotnode
  src/Main.cpp
  src/Logger.cpp
  src/RobotController.cpp

  # EGM
  src/egm/EGMThread.cpp
  src/egm/PracticalSocket.cpp

  # Common functions for services
  src/services/Common.cpp

  # Info services
  src/services/Info/Ping.cpp
  src/services/Info/GetCartesian.cpp
  src/services/Info/GetEGMMode.cpp
  src/services/Info/GetFK.cpp
  src/services/Info/GetGripperPosition.cpp
  src/services/Info/GetGripperStatus.cpp
  src/services/Info/GetIK.cpp
  src/services/Info/GetJoints.cpp
  src/services/Info/GetRobotAngle.cpp
  src/services/Info/SetDefaults.cpp
  src/services/Info/SetInertia.cpp
  src/services/Info/SetIOSignal.cpp
  src/services/Info/SetMaxSpeed.cpp
  src/services/Info/SetTool.cpp
  src/services/Info/SetWorkObject.cpp
  src/services/Info/SetZone.cpp

  # Motion services
  src/services/Motion/AddToCartesianBuffer.cpp
  src/services/Motion/ClearCartesianBuffer.cpp
  src/services/Motion/ExecuteCartesianBuffer.cpp
  src/services/Motion/ExecuteSynchroCartesianBuffer.cpp
  src/services/Motion/GetCartesianBufferSize.cpp
  src/services/Motion/AddToJointBuffer.cpp
  src/services/Motion/ClearJointBuffer.cpp
  src/services/Motion/ExecuteJointBuffer.cpp
  src/services/Motion/ExecuteSynchroJointBuffer.cpp
  src/services/Motion/GetJointBufferSize.cpp
  src/services/Motion/SetCartesian.cpp
  src/services/Motion/SetEGMMode.cpp
  src/services/Motion/SetJoints.cpp
  src/services/Motion/SetMaxAcceleration.cpp
  src/services/Motion/SetMotionSupervision.cpp
  src/services/Motion/Stop.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(abb_robotnode PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(abb_robotnode ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} ${PROTO_HDRS})
add_dependencies(abb_robotnode abb_robotnode_generate_messages_cpp)
