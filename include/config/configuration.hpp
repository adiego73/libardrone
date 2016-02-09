#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <string>

namespace robot
{

struct PIDValues
{
    float Kp;
    float Kd;
    float Ki;
    float Integrator;
    float Derivator;
    float P_limit;
    float I_max;
    float I_min;
};

struct PIDConfig
{
    PIDValues Roll;
    PIDValues Pitch;
    PIDValues Yaw;
    PIDValues Altitude;
};

struct RobotConfig
{
    PIDConfig PID;
    std::string address;

};

}
#endif                                                      // CONFIGURATION_HPP
