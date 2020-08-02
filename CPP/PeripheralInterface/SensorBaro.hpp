#ifndef __SENSORGPS__H
#define __SENSORGPS__H

#include "MsgBus/MsgType.hpp"

namespace FC{

class SensorBaro{
public:
    void setBaro(float pressure, float temperature);
private:
    struct Barometer barometer;
}

}


#endif