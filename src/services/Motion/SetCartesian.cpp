#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(SetCartesian)
{
  if(egmMode == EGM_CART_POS_AUTO) {

  } else if(egmMode == EGM_OFF) {
    // If EGM is off, send command via TCP/IP
    int randNumber = generateRandNumber();
    if(prepareCartesian(motionMsg, 1, randNumber, req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz)) {
      if(sendMotion(randNumber)) {
        int ok, idCode;
        sscanf(motionReply, "%*d %d %d", &idCode, &ok);
        res.success = (bool) ok;
        res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
      } else {
        res.success = false;
        res.msg = "No answer received.";
      }
    } else {
      res.success = false;
      res.msg = "Sent pose is not valid.";
    }
  } else {
    res.success = false;
    res.msg = "Current EGM mode is incompatible with this task.";
  }
  return true;
}
