#ifndef PID_Z_H
#define PID_Z_H

#include "pid.hpp"

namespace robot
{

class PID_Z : public PID
{
    public:
        PID_Z( float kp, float kd, float P_limit, float set_point );
        ~PID_Z();

        void setPoint( float set_point ) override;
        float update( float current_value, float change, long int dt )  override;
};

}
#endif // PID_Z_H
