#ifndef CASCADE_CONTROLLER_H
#define CASCADE_CONTROLLER_H

#include "quadcopter.h"

namespace quad {

// Mirrors the control law in matlab_pid_sim/main_*.m exactly, so the C++ port can be validated against MATLAB.
struct CascadeGains {
    double pos_sat = 3.0;
    double gain_pos = 0.24;
    double gain_vel = 0.1;
    double angle_max = 0.3;

    double takeoff_height = 0.5;
    double Kp_z_takeoff = 0.45;
    double Kp_z = 0.8;
    double Kd_z = 0.3;

    double Kp_phi = 0.01,   Ki_phi = 0.01,   Kd_phi = 0.0028;
    double Kp_theta = 0.013, Ki_theta = 0.01, Kd_theta = 0.002;
    double Kp_psi = 0.004,  Kd_psi = 0.00012;

    double int_max = 0.05;
};

class CascadeController {
public:
    CascadeController() = default;
    explicit CascadeController(const CascadeGains& gains) : gains_(gains) {}

    Control update(const Eigen::Vector4d& ref, const State& state);

    void setGains(const CascadeGains& gains) { gains_ = gains; }
    void reset();

private:
    CascadeGains gains_;
    double int_phi_ = 0.0;
    double int_theta_ = 0.0;
    double int_psi_ = 0.0;
};

} // namespace quad

#endif
