#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace quad {

class PID {
public:
    PID();
    PID(double Kp, double Ki, double Kd, double Ts, double u_min, double u_max);

    double update(double ref, double meas);
    
    // Updates PID using an explicitly provided rate for the D-term 
    // instead of numerically differentiating the error.
    double update_with_rate(double ref, double meas, double rate);

    void setGains(double Kp, double Ki, double Kd);
    void setLimits(double u_min, double u_max);
    void reset();

    double getIntegral() const { return integral_; }

private:
    double Kp_, Ki_, Kd_;
    double Ts_;
    double u_min_, u_max_;
    double integral_;
    double prev_error_;
};

} // namespace quad

#endif
