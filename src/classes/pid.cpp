#include "classes/pid.hpp"

using namespace robot;

PID::PID( float kp, float ki, float kd, int D, int I, float P_limit, int I_max, int I_min, float set_point, float power ) :
    Kp( kp ), Ki( ki ), Kd( kd ),
    Derivator( D ), Integrator( I ),
    P_limit( P_limit ), Integrator_max( I_max ), Integrator_min( I_min ),
    set_point( set_point ), power( power )
{
    this->error = 0.0f;
    this->last_error = 0.0f;
    this->last_value = 0.0f;

}

PID::~PID()
{

}

float PID::getKd()
{
    return this->D_value;
}

float PID::getKi()
{
    return this->I_value;
}

float PID::getKp()
{
    return this->P_value;
}

void PID::reset()
{
    this->Derivator = 0;
    this->Integrator = 0;
}

void PID::setPoint( float set_point )
{
    this->set_point = set_point;
    this->reset();
}

float PID::update( float current_value, float change, long int dt )
{
    this->error = this->set_point - current_value;

    this->P_value = this->Kp * this->error;

    if( this->error > 0 )
    {
        this->I_value = this->Integrator * this->Ki;
    }
    else
    {
        this->I_value = this->Integrator * this->Ki * 0.5;
    }

    this->D_value = this->Kd * ( this->error - this->Derivator );

    this->Derivator = this->error;
    this->Integrator = this->Integrator + this->error / 200;

    if( this->Integrator > this->Integrator_max )
    {
        this->Integrator = this->Integrator_max;
    }
    else if( this->Integrator < this->Integrator_min )
    {
        this->Integrator = this->Integrator_min;
    }

    this->last_error = this->error;
    this->last_value = current_value;

    float pid_value = this->P_value + this->I_value + this->D_value;

    return pid_value;
}

