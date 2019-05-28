#include "abb_robotnode/RobotController.hpp"

int RobotController::setMaxAcceleration(double acc, double deacc) {
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d %08.2lf %08.2lf #", 15, randNumber, acc, deacc);

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    return ((bool) ok ? 1 : -1);
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetMaxAcceleration)
{
  SERVICE_CHECK_EGM_OFF()
  
  int result = setMaxAcceleration(req.acc, req.deacc);
  SERVICE_RESPONSE_FROM_RESULT()
}
