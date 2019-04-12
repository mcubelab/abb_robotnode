#ifndef _ABB_ROBOTNODE_SERVICES_HPP_
#define _ABB_ROBOTNODE_SERVICES_HPP_

// Info services
#include "abb_robotnode/Service_Ping.h"
#include "abb_robotnode/Service_GetCartesian.h"
#include "abb_robotnode/Service_GetEGMMode.h"
#include "abb_robotnode/Service_GetFK.h"
#include "abb_robotnode/Service_GetGripperPosition.h"
#include "abb_robotnode/Service_GetGripperStatus.h"
#include "abb_robotnode/Service_GetIK.h"
#include "abb_robotnode/Service_GetJoints.h"
#include "abb_robotnode/Service_GetRobotAngle.h"
#include "abb_robotnode/Service_SetDefaults.h"
#include "abb_robotnode/Service_SetInertia.h"
#include "abb_robotnode/Service_SetIOSignal.h"
#include "abb_robotnode/Service_SetMaxAcceleration.h"
#include "abb_robotnode/Service_SetMaxSpeed.h"
#include "abb_robotnode/Service_SetMotionSupervision.h"
#include "abb_robotnode/Service_SetTool.h"
#include "abb_robotnode/Service_SetWorkObject.h"
#include "abb_robotnode/Service_SetZone.h"

// Motion services
#include "abb_robotnode/Service_AddToCartesianBuffer.h"
#include "abb_robotnode/Service_ClearCartesianBuffer.h"
#include "abb_robotnode/Service_ExecuteCartesianBuffer.h"
#include "abb_robotnode/Service_ExecuteSynchroCartesianBuffer.h"
#include "abb_robotnode/Service_GetCartesianBufferSize.h"
#include "abb_robotnode/Service_AddToJointBuffer.h"
#include "abb_robotnode/Service_ClearJointBuffer.h"
#include "abb_robotnode/Service_ExecuteJointBuffer.h"
#include "abb_robotnode/Service_ExecuteSynchroJointBuffer.h"
#include "abb_robotnode/Service_GetJointBufferSize.h"
#include "abb_robotnode/Service_SetCartesian.h"
#include "abb_robotnode/Service_SetEGMMode.h"
#include "abb_robotnode/Service_SetJoints.h"
#include "abb_robotnode/Service_Stop.h"


// Common definitions
#define SERVICE_CALLBACK_DEC(X) bool ServiceCallback_##X(abb_robotnode::Service_##X::Request& req, abb_robotnode::Service_##X::Response& res);
#define SERVICE_CALLBACK_DEF(X) bool RobotController::ServiceCallback_##X(abb_robotnode::Service_##X::Request& req, abb_robotnode::Service_##X::Response& res)
#define ADVERTISE_SERVICE(X) advertiseService(name+"_"#X, &RobotController::ServiceCallback_##X,
#define END_SERVICE() );

#define SERVICE_AUX_LIST() \
void parseJoints(char* reply, int& idCode, int& ok, std::vector<double>& angles); \
bool prepareJoints(char* msg, int idCode, int randNumber, std::vector<double>& joints); \
bool prepareJoints(char* msg, int idCode, int randNumber, std::vector<double>& joints, double extra); \
bool prepareCartesian(char* msg, int idCode, int randNumber, double x, double y, double z, double q0, double qx, double qy, double qz); \
bool prepareCartesian(char* msg, int idCode, int randNumber, double x, double y, double z, double q0, double qx, double qy, double qz, double extra); \
int setDefaults(); \
int setInertia(double m, double cgx, double cgy, double cgz, double ix, double iy, double iz); \
int setMaxSpeed(double tcp, double ori, double joints); \
int setMaxAcceleration(double acc, double deacc); \
int setMotionSupervision(double val); \
int setTool(double x, double y, double z, double q0, double qx, double qy, double qz); \
int setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz); \
int setZone(int z);

// All services should be added:
// - below, in the list of service callbacks and advertised services,
// - .srv file in srv/ folder,
// - .cpp file in src/services/ folder,
// - CMakeLists.txt: add_service_files (.srv file) / add_executable (.cpp file),
// - include .h file on top of this file.

#define SERVICE_FUNCTION_LIST() \
SERVICE_CALLBACK_DEC(Ping) \
SERVICE_CALLBACK_DEC(GetCartesian) \
SERVICE_CALLBACK_DEC(GetEGMMode) \
SERVICE_CALLBACK_DEC(GetFK) \
SERVICE_CALLBACK_DEC(GetIK) \
SERVICE_CALLBACK_DEC(SetIOSignal) \
SERVICE_CALLBACK_DEC(GetGripperPosition) \
SERVICE_CALLBACK_DEC(GetGripperStatus) \
SERVICE_CALLBACK_DEC(GetJoints) \
SERVICE_CALLBACK_DEC(GetRobotAngle) \
SERVICE_CALLBACK_DEC(AddToCartesianBuffer) \
SERVICE_CALLBACK_DEC(ClearCartesianBuffer) \
SERVICE_CALLBACK_DEC(ExecuteCartesianBuffer) \
SERVICE_CALLBACK_DEC(ExecuteSynchroCartesianBuffer) \
SERVICE_CALLBACK_DEC(GetCartesianBufferSize) \
SERVICE_CALLBACK_DEC(AddToJointBuffer) \
SERVICE_CALLBACK_DEC(ClearJointBuffer) \
SERVICE_CALLBACK_DEC(ExecuteJointBuffer) \
SERVICE_CALLBACK_DEC(ExecuteSynchroJointBuffer) \
SERVICE_CALLBACK_DEC(GetJointBufferSize) \
SERVICE_CALLBACK_DEC(SetCartesian) \
SERVICE_CALLBACK_DEC(SetDefaults) \
SERVICE_CALLBACK_DEC(SetEGMMode) \
SERVICE_CALLBACK_DEC(SetInertia) \
SERVICE_CALLBACK_DEC(SetJoints) \
SERVICE_CALLBACK_DEC(SetMaxAcceleration) \
SERVICE_CALLBACK_DEC(SetMaxSpeed) \
SERVICE_CALLBACK_DEC(SetMotionSupervision) \
SERVICE_CALLBACK_DEC(SetTool) \
SERVICE_CALLBACK_DEC(SetWorkObject) \
SERVICE_CALLBACK_DEC(SetZone) \
SERVICE_CALLBACK_DEC(Stop)

#define ADVERTISE_SERVICES() \
ADVERTISE_SERVICE(Ping) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetCartesian) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetEGMMode) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetFK) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetGripperPosition) IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetGripperStatus) IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetIK) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetJoints) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetRobotAngle) IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(AddToCartesianBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ClearCartesianBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ExecuteCartesianBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ExecuteSynchroCartesianBuffer) IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetCartesianBufferSize) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(AddToJointBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ClearJointBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ExecuteJointBuffer) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(ExecuteSynchroJointBuffer) IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(GetJointBufferSize) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetCartesian) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetDefaults) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetEGMMode) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetInertia) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetIOSignal) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetJoints) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetMaxAcceleration) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetMaxSpeed) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetMotionSupervision) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetTool) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetWorkObject) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(SetZone) IRB120|IRB1600|IRB14000 END_SERVICE() \
ADVERTISE_SERVICE(Stop) IRB120|IRB1600|IRB14000 END_SERVICE()


#endif
