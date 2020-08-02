#ifndef __SENSORGYRO__H
#define __SENSORGYRO__H

#include "MsgBus/MsgType.hpp"

namespace FC{

class SensorGyro{
public:
    void setGyro(float x, float y, float z);
private:
    struct BodyAngularVelocity bodyAngularVelocity;
}


}


#endif