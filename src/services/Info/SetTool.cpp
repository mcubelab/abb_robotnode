#include "abb_robotnode/RobotController.hpp"

int RobotController::setTool(double x, double y, double z, double q0, double qx, double qy, double qz) {
  if(x == currentTool[0] && y == currentTool[1] && z == currentTool[2]
     && q0 == currentTool[3] && qx == currentTool[4] && qy == currentTool[5] && qz == currentTool[6])
    return 1;

  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf #", 6, randNumber, x, y, z, q0, qx, qy, qz);

  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    if((bool) ok) {
      currentTool[0] = x;
      currentTool[1] = y;
      currentTool[2] = z;
      currentTool[3] = q0;
      currentTool[4] = qx;
      currentTool[5] = qy;
      currentTool[6] = qz;
      return 1;
    }
    return -1;
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetTool)
{
  int result = setTool(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz);
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
