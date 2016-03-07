#ifndef PID_RP_H
#define PID_RP_H

#include <vector>

#include "pid.hpp"

namespace robot
{

class PID_RP : public PID
{
    public:
        PID_RP( float kp, float ki, float kd, float P_limit, long I_max );
        ~PID_RP();

        void setPoint( float set_point ) override;
	void updateSamePoint( float set_point );
        float update( float current_value, float change, long int dt ) override;

    private:
        std::vector<float> Integrators;
        void integratorIncrease( float error, long max, float change );
};


}
#endif // PID_RP_H
