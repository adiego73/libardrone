#ifndef PID_H
#define PID_H
#include <message_client.hpp>

namespace robot
{

class PID
{
    public:
        PID( float kp, float ki, float kd, int D, int I, float P_limit, int I_max, int I_min, float set_point, float power );
        ~PID();

        virtual float update( float current_value, float change, long dt );
        virtual void setPoint( float set_point );
        virtual void reset();
        virtual float getKp();
        virtual float getKi();
        virtual float getKd();

    protected:
        float I_value;
        float D_value;
        float P_value;

        float Kp;
        float Ki;
        float Kd;

        float P_limit;
        float set_point;
        float error;
        float power;

        float last_error;
        float last_value;

        int Derivator;
        float Integrator;

        int Integrator_max;
        int Integrator_min;
};

}
#endif // PID_H
