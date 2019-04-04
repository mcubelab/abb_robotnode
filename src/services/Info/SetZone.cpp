#include "abb_robotnode/RobotController.hpp"
#include <abb_robotnode/Zones.hpp>

int RobotController::setZone(int z) {
  if(z == currentZone)
    return 1;

  int randNumber = generateRandNumber();
  bool fine = (z == ZONE_FINE);
  sprintf(infoMsg, "%.2d %.3d %.1d %.2lf %.2lf %.2lf #", 9, randNumber, fine, zoneData[z].p_tcp, zoneData[z].p_ori, zoneData[z].ori);

  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    if((bool) ok) {
      currentZone = z;
      return 1;
    }
    return -1;
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetZone)
{
  int result = setZone(req.mode);
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
