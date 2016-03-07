#include "classes/pid_rp.hpp"
#include <iostream>
using namespace robot;

PID_RP::PID_RP( float kp, float ki, float kd, float P_limit, long I_max ):
    PID( kp, ki, kd, P_limit, I_max, 0 )
{

}

PID_RP::~PID_RP()
{

}

void PID_RP::setPoint( float set_point )
{
    PID::setPoint( set_point );

    this->Integrators.clear();
    this->last_error = set_point;
    this->error = 0.0f;
}

void PID_RP::updateSamePoint( float set_point )
{
    this->set_point = set_point;

}

void PID_RP::integratorIncrease( float error, long max, float change )
{
    if( error < 0 && this->Integrator > 0 )
    {
       this->Integrator = std::max( 0.0, this->Integrator + 14.0 * error );
    }
    else if( error > 0 && this->Integrator < 0 )
    {
        this->Integrator = std::min( 0.0, this->Integrator + 14.0 * error );
    }
    else if( change < 0 && this->Integrator > 0 && error > 0 ) //si me estoy acercando
    {
      this->Integrator = std::max( 0.0f, float( this->Integrator + error ) );
      
      
    }
    else if( change > 0 && this->Integrator < 0 && error < 0 ) //si me estoy acercando
    {
      this->Integrator = std::min( 0.0f, float( this->Integrator + error ) );
      std::cout<<"integrator: "<<this->Integrator <<std::endl;
      
    }
    else
    {
        this->Integrator += error;
    }
    
    if( this->Integrator > max )
    {
      this->Integrator  = max;
    }
    else if( this->Integrator < -max )
    {
      this->Integrator  = -max;
    }
    

}

float PID_RP::update( float current_value, float change, long int dt )
{
    std::size_t COUNT = 2;
    float changes = 0.0f;

    this->error = current_value;
    float error_difference = this->error - this->last_error;          //si es positivo me alejo, si es negativo me acerco

    this->Integrators.push_back( error_difference / dt );

    if( this->Integrators.size() > COUNT )
    {
        this->Integrators.erase( this->Integrators.begin() );
    }

    for( std::size_t i = 0; i < this->Integrators.size(); i++ )
    {
        changes += this->Integrators[i];
    }

    changes = changes / this->Integrators.size();

    this->integratorIncrease( this->error * dt, this->Integrator_max, changes );

    this->P_value = this->Kp * this->error;
    this->I_value = this->Integrator * this->Ki;
    this->D_value = this->Kd * change;                      //usp el del robot

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

