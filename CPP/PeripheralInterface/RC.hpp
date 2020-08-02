#ifndef __RC__H
#define __RC__H

#include "MsgBus/MsgType.hpp"

class RC{
public:
    setRC(uint16_t *ch, uint8_t len);
private:
    struct Controller controller;
}

#endif