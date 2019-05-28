#include "abb_robotnode/RobotController.hpp"

int RobotController::setMaxSpeed(double tcp, double ori, double joints) {
  maxTcpSpeed = tcp;
  maxOriSpeed = ori;
  maxJointSpeed = joints;

  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d %08.1lf %08.2lf #", 8, randNumber, maxTcpSpeed, maxOriSpeed);

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    return ((bool) ok ? 1 : -1);
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetMaxSpeed)
{
  SERVICE_CHECK_EGM_OFF()

  int result = setMaxSpeed(req.tcp, req.ori, req.joints);
  SERVICE_RESPONSE_FROM_RESULT()
}
