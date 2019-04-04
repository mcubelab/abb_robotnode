#include "abb_robotnode/RobotController.hpp"

int RobotController::setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz) {
  if(x == currentWobj[0] && y == currentWobj[1] && z == currentWobj[2]
     && q0 == currentWobj[3] && qx == currentWobj[4] && qy == currentWobj[5] && qz == currentWobj[6])
    return 1;

  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf #", 7, randNumber, x, y, z, q0, qx, qy, qz);
  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    if((bool) ok) {
      currentWobj[0] = x;
      currentWobj[1] = y;
      currentWobj[2] = z;
      currentWobj[3] = q0;
      currentWobj[4] = qx;
      currentWobj[5] = qy;
      currentWobj[6] = qz;
      return 1;
    }
    return -1;
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetWorkObject)
{
  int result = setWorkObject(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz);
  if(result == 1) {
    res.success = true;
    res.msg = "Ok.";
  } else if(result == -1) {
    res.success = false;
    res.msg = "Wrong answer from robot.";
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
