#include "PID.h"

PID::PID()
{
    _Kp = 100;
    _Ki = 0;//20
    _Kd = 0;
    _min = -7000;
    _max = 7000;

    _prev_error = 0;
    _integral = 0;
    _setpoint = 0;
}

float PID::calculate( float w_rocket, float dt )
{
    // Calculate error
    float error = w_rocket - _setpoint;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _prev_error) / dt;
    float Dout = _Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    // This should protect against integral windup
    if( output > _max ){
        output = _max;
    }
    else if( output < _min ){
        output = _min;
    }

    // Save error to previous error
    _prev_error = error;

    return output;
}
