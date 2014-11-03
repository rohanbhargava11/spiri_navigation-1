#include <utils/pid.h>

PID::PID() :
    kp(2.0), ki(0.02), kd(0.2),
    max_err(10000.0),
    last_err(0.0),
    acc(0.0), max_acc(10000.0)
{ }

PID::PID(double kp, double ki, double kd, double max_err, double max_acc, bool angular) :
    kp(kp), ki(ki), kd(kd),
    max_err(max_err),
    last_err(0.0),
    acc(0.0), max_acc(max_acc), angular(angular)
{ }

double PID::update(double err, double dt)
{
    if (angular)
    {
        while (fabs(err) > 2*M_PI)
        {
            if (err > 0)
                err -= 2*M_PI;
            else
                err += 2*M_PI;
        }
        if (err > M_PI)
            err = -2*M_PI + err;
        else if (err < -M_PI)
            err = 2*M_PI - err;
    }
            
    if (err > max_err)
        err = max_err;
    else if (err < -max_err)
        err = -max_err;
    
    double d_err = (err - last_err) / dt;
    
    acc += err;
    if (acc > max_acc)
        acc = max_acc;
    else if (acc < -max_acc)
        acc = -max_acc;
        
    last_err = err;
    double sig_out = kp*err - kd*d_err + ki*acc;
    
    return sig_out;    
}
