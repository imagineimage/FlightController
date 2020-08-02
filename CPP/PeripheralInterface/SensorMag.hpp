#ifndef __SENSORMAG__H
#define __SENSORMAG__H

#include "MsgBus/MsgType.hpp"

namespace FC{

class SensorMag{
public:
    void setMag(float x, float y, float z);
    void calibration();
private:
    struct BodyMag bodyMag;
    float bias[3];
    float scale[3];
}

}


#endif