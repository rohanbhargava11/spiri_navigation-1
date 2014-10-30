#include <utils/pid.h>

PID::PID() :
    setpoint(0.0),
    kp(2.0), ki(0.02), kd(0.2),
    max_err(10000.0),
    last_err(0.0),
    acc(0.0), max_acc(10000.0)
{ }

PID::PID(double setpoint) :
    setpoint(setpoint),
    kp(2.0), ki(0.02), kd(0.2),
    max_err(10000.0),
    last_err(0.0),
    acc(0.0), max_acc(10000.0)
{ }

PID::PID(double setpoint, double kp, double ki, double kd, double max_err, double max_acc) :
    setpoint(setpoint),
    kp(kp), ki(ki), kd(kd),
    max_err(max_err),
    last_err(0.0),
    acc(0.0), max_acc(max_acc)
{ }

double PID::update(double current_state, double dt)
{
    double err = setpoint - current_state;
    
    double d_err = (err - last_err) / dt;
    
    acc += err;
    if (acc > max_err)
        acc = max_err;
    else if (acc < -max_err)
        acc = -max_err;
        
    double sig_out = kp*err - kd*d_err + ki*acc;
    return sig_out;    
}
