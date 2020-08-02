#ifndef __SENSORGPS__H
#define __SENSORGPS__H

#include "MsgBus/MsgType.hpp"

namespace FC{

class SensorGPS{
public:
    void setGPS(double lat, double lon, double alt
                float vel, float direction, float hdop, float vdop, 
                uint8_t numSat, uint8_t fixType, uint64_t UtcUsec);
private:
    struct GPS gps;
}

}


#endif