#include "abb_robotnode/RobotController.hpp"

int RobotController::setInertia(double m, double cgx, double cgy, double cgz, double ix, double iy, double iz) {
  if(m == currentInertia[0] && cgx == currentInertia[1] && cgy == currentInertia[2] && cgz == currentInertia[3]
     && ix == currentInertia[4] && iy == currentInertia[5] && iz == currentInertia[6])
    return 1;

  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d %+08.5lf %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf #", 14, randNumber, m, cgx, cgy, cgz, ix, iy, iz);

  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    if((bool) ok) {
      currentInertia[0] = m;
      currentInertia[1] = cgx;
      currentInertia[2] = cgy;
      currentInertia[3] = cgz;
      currentInertia[4] = ix;
      currentInertia[5] = iy;
      currentInertia[6] = iz;
      return 1;
    }
    return -1;
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetInertia)
{
  int result = setInertia(req.m, req.cgx, req.cgy, req.cgz, req.ix, req.iy, req.iz);
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
