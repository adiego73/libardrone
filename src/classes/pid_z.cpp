#include "classes/pid_z.hpp"

using namespace robot;

PID_Z::PID_Z( float kp, float kd, float P_limit, float set_point ) :
    PID( kp, 0.0f, kd, 0, 0, P_limit, 0, 0, set_point, 0 )
{

}

PID_Z::~PID_Z()
{

}

void PID_Z::setPoint( float set_point )
{
    PID::setPoint( set_point );
    this->last_error = 0;
    this->error = 0;
}

float PID_Z::update( float current_value, float change, long int dt )
{
    this->error = this->set_point - current_value;

    change = this->error - this->last_error;

    this->P_value = this->Kp * this->error;
    this->D_value = this->Kd * change / dt;

    this->last_error = this->error;

    float total = this->P_value + this->D_value;

    if( total > this->P_limit )
    {
        total = this->P_limit;
    }
    else if( total < ( this->P_limit * -1 ) )
    {
        total = this->P_limit * -1;
    }

    return total;
}
