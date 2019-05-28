#include "abb_robotnode/RobotController.hpp"

int RobotController::setMotionSupervision(double val) {
  if(val <= 0 || val > 300) {
    ROS_WARN("%s: Invalid value sent to SetMotionSupervision.", msgHeader.c_str());
    return 0;
  }
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d %08.1lf #", 5, randNumber, val);

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    return ((bool) ok ? 1 : -1);
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetMotionSupervision)
{
  SERVICE_CHECK_EGM_OFF()

  int result = setMotionSupervision(req.supervision);
  SERVICE_RESPONSE_FROM_RESULT()

  return true;
}
