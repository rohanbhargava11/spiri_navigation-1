

class PID
{
  public:
    PID();
    PID(double setpoint);
    PID(double setpoint, double kp, double ki, double kd, double max_err, double max_acc);
    
    ~PID() { }
    
    double update(double current_state, double dt);
    
    void setSetpoint(double new_setpoint) {setpoint = new_setpoint; acc = 0;}
  protected:
    double setpoint;
    double kp, ki, kd;
    double max_err;
    double last_err;
    double acc, max_acc;
    
};
