#include <math.h>

class PID
{
  public:
    PID();
    PID(double kp, double ki, double kd, double max_err, double max_acc, bool angular);
    
    ~PID() { }
    
    double update(double err, double dt);
    double getLastError() { return last_err; }
    
  protected:
    double kp, ki, kd;
    double max_err;
    double last_err;
    double acc, max_acc;
    bool angular;
    
};
