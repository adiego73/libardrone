#ifndef PID_H
#define PID_H

namespace robot
{

class PID
{
    public:
        PID( float kp, float ki, float kd, float P_limit, long I_max, long I_min );
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

        float last_error;
        float last_value;

        int Derivator;
        long Integrator;

        long Integrator_max;
        long Integrator_min;
};

}
#endif // PID_H
