#ifndef PID_Y_H
#define PID_Y_H

#include <vector>

#include "pid.hpp"

namespace robot
{

class PID_Y : public PID
{
    public:
        PID_Y( float kp, float ki, float kd, int D, int I, float P_limit, int I_max, int I_min, float set_point, float power );
        ~PID_Y();
        
        void setPoint( float set_point ) override;
        float update( float current_value, float change, long int dt ) override;

    private:
        std::vector<float> Integrators;
};

}
#endif // PID_Y_H
