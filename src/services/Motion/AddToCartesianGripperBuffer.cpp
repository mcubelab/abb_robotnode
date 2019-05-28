#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(AddToCartesianGripperBuffer)
{
  SERVICE_CHECK_EGM_OFF()
  
  int randNumber = generateRandNumber();
  if(!prepareCartesian(motionMsg, 30, randNumber, req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz, req.gripperPos)) {
    res.success = false;
    res.msg = "Sent pose is not valid.";
    return true;
  }

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    res.success = (bool) ok;
    res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
