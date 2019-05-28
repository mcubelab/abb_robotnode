#include "abb_robotnode/RobotController.hpp"
#include <abb_robotnode/Zones.hpp>

int RobotController::setZone(int z) {
  if(z == currentZone)
    return 1;

  int randNumber = generateRandNumber();
  bool fine = (z == ZONE_FINE);
  sprintf(motionMsg, "%.2d %.3d %.1d %.2lf %.2lf %.2lf #", 9, randNumber, fine, zoneData[z].p_tcp, zoneData[z].p_ori, zoneData[z].ori);

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
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
  SERVICE_CHECK_EGM_OFF()
  
  int result = setZone(req.mode);
  SERVICE_RESPONSE_FROM_RESULT()
}
