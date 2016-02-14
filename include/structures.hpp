#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP

namespace robot
{

#ifndef MAX_ANGLE
#define MAX_ANGLE 180
#endif

struct Point
{
    float x;
    float y;
    float z;
};

struct Velocity
{
    double x;
    double y;
    double z;
};

}
#endif                                                      // STRUCTURES_HPP
