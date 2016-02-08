#include "classes/pid_y.hpp"
using namespace robot;

PID_Y::PID_Y( float kp, float ki, float kd, int D, int I, float P_limit, int I_max, int I_min, float set_point, float power ) :
    PID( kp, ki, kd, D, I, P_limit, I_max, I_min, set_point, power )
{

}

PID_Y::~PID_Y()
{

}

void PID_Y::setPoint( float set_point )
{
    PID::setPoint( set_point );
    this->Integrators.clear();

    this->last_error = 0.0f;
    this->error = 0.0f;
}

float PID_Y::update( float current_value, float change, long int dt )
{
    float changes = 0;
    std::size_t COUNT = 3;

    this->error = this->set_point - current_value;

    if( this->error > 180 || this->error < -180 )
    {
        if( current_value > 0 )
        {
            current_value = -180 - ( 180 - current_value );
        }
        else
        {
            current_value = 180 + ( 180 + current_value );
        }

        this->error = this->set_point - current_value;
    }

    change = this->error - this->last_error;


    this->Integrators.push_back( change );

    if( this->Integrators.size() > COUNT )
    {
        this->Integrators.erase( this->Integrators.begin() );
    }

    for( std::size_t i = 0; i < this->Integrators.size(); i++ )
    {
        changes += this->Integrators[i];
    }

    changes = changes / this->Integrators.size();

    //aumento Kd   si me alejo

    this->Derivator = this->error;
    this->Integrator += this->error;

    if( this->Integrator > this->Integrator_max )
    {
        this->Integrator = this->Integrator_max;
    }
    else if( this->Integrator < this->Integrator_min )
    {
        this->Integrator = this->Integrator_min;
    }

    //aumento Kp si me alejo
    this->P_value = this->Kp * this->error;

    if( this->P_value > this->P_limit )
    {
        this->P_value = this->P_limit;
    }
    else if( this->P_value < ( this->P_limit * -1 ) )
    {
        this->P_value = this->P_limit * -1;
    }

    this->I_value = this->Integrator * this->Ki;
    this->D_value = this->Kd * changes;

    this->last_error = this->error;

    float total = this->P_value + this->I_value + this->D_value;

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
