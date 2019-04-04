#include <abb_robotnode/Services.hpp>
#include "abb_robotnode/RobotController.hpp"

bool RobotController::prepareCartesian(char* msg, int idCode, int randNumber, double x, double y, double z, double q0, double qx, double qy, double qz)
{
  // Validate that quaternion is valid
  if(abs(q0*q0+qx*qx+qy*qy+qz*qz-1) < 0.001) {
    sprintf(msg, "%.2d %.3d %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf #", idCode, randNumber, x, y, z, q0, qx, qy, qz);
    return true;
  }
  return false;
}

bool RobotController::prepareCartesian(char* msg, int idCode, int randNumber, double x, double y, double z, double q0, double qx, double qy, double qz, double extra)
{
  // Validate that quaternion is valid
  if(abs(q0*q0+qx*qx+qy*qy+qz*qz-1) < 0.001) {
    sprintf(msg, "%.2d %.3d %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf #", idCode, randNumber, x, y, z, q0, qx, qy, qz, extra);
    return true;
  }
  return false;
}

bool RobotController::prepareJoints(char* msg, int idCode, int randNumber, std::vector<double>& joints)
{
  if(model == IRB14000) {
    // 7-DOF models
    if(joints.size() != 7) return false;
    sprintf(msg, "%.2d %.3d %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf #", idCode, randNumber, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);
  } else {
    // 6-DOF models
    if(joints.size() != 6) return false;
    sprintf(msg, "%.2d %.3d %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf #", idCode, randNumber, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  }
  return true;
}

bool RobotController::prepareJoints(char* msg, int idCode, int randNumber, std::vector<double>& joints, double extra)
{
  if(model == IRB14000) {
    // 7-DOF models
    if(joints.size() != 7) return false;
    sprintf(msg, "%.2d %.3d %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf #", idCode, randNumber, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6], extra);
  } else {
    // 6-DOF models
    if(joints.size() != 6) return false;
    sprintf(msg, "%.2d %.3d %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf #", idCode, randNumber, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], extra);
  }
  return true;
}

void RobotController::parseJoints(char* reply, int& idCode, int& ok, std::vector<double>& joints)
{
  std::vector<double> angles;
  if(model == IRB14000) {
    // 7-DOF models
    angles = std::vector<double>(7);
    sscanf(reply, "%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf", &idCode, &ok, &angles[0], &angles[1], &angles[2], &angles[3], &angles[4], &angles[5], &angles[6]);
  } else {
    // 6-DOF models
    angles = std::vector<double>(6);
    sscanf(reply, "%*d %d %d %*f %lf %lf %lf %lf %lf %lf", &idCode, &ok, &angles[0], &angles[1], &angles[2], &angles[3], &angles[4], &angles[5]);
  }
  joints = angles;
}
