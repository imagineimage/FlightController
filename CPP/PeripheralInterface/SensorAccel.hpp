#ifndef __SENSORACCEL__H
#define __SENSORACCEL__H

#include "main.h"
#include "MsgBus/MsgType.hpp"

namespace FC{

class SensorAccel{
public:
	SensorAccel() = default;
    void setAccel(float x, float y, float z);
private:
    struct BodyAccel bodyAccel{};
};

SensorAccel sensorAccel;

}


#endif
