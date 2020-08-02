#ifndef __MSGBUS__H
#define __MSGBUS__H

// #define SIMPLE_FUNC_IMPL(ret, fname, var) \
//            ret get##fname() \
//            { \
//                      return var; \
//            } \
//            void set##fname(ret tmp) \
//            { \
//                      var = tmp; \
//            }

#define SIMPLE_FUNC_IMPL(ret, fname, var) \
            bool get##fname(ret *tmp) \
            { \
                if(tmp->timestamp != var.timestamp){\
                    *tmp = var;\
                    return true; \
                }\
                return false\
            } \
            void set##fname(const ret &tmp) \
            { \
                var = tmp; \
            }

#include "MsgType.hpp"

namespace FC{


class MsgBus {
public:
    SIMPLE_FUNC_IMPL(BodyAccel, BodyAccel, bodyAccel);
    SIMPLE_FUNC_IMPL(BodyAngularVelocity, BodyAngularVelocity, bodyAngularVelocity);
    SIMPLE_FUNC_IMPL(BodyMag, BodyMag, bodyMag);
    SIMPLE_FUNC_IMPL(GPS, GPS, gps);
    SIMPLE_FUNC_IMPL(Barometer, Barometer, barometer);
    SIMPLE_FUNC_IMPL(Controller, Controller, controller);

    
    SIMPLE_FUNC_IMPL(Controller, Controller, controller);
    SIMPLE_FUNC_IMPL(Controller, Controller, controller);

private:
    static struct BodyAccel bodyAccel{};
    static struct BodyAngularVelocity bodyAngularVelocity{};
    static struct BodyMag bodyMag{};
    static struct GPS gps{};
    static struct Barometer barometer{};
    static struct Controller controller{};

    static struct Attitude attitude{};
    static struct NedAccel nedAccel{};
}

}

#endif