#ifndef __MODULEAHRS__H
#define __MODULEAHRS__H

#include "MsgType.hpp"

class ModuleAHRS{
public:
    void init();
    void main();
private:
    /* input */
    struct BodyAccel bodyAccel{};
    struct BodyAngularVelocity bodyAngularVelocity{};
    struct BodyMag bodyMag{};

    /* output */
    struct Attitude attitude{};
    struct NedAccel nedAccel{};

    uint64_t lastUpdate{};
    float beta{0.1f};
    
    void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void calNedAccel();

    float invSqrt(float x);
}

extern"C" {
void moduleAhrsMain();
}

#endif