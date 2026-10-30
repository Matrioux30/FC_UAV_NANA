#include "cascade_controller.h"
#include <algorithm>
#include <cmath>

namespace quad {

namespace {

double sgn(double v) {
    return static_cast<double>((v > 0.0) - (v < 0.0));
}

void integrateWithAntiWindup(double& integral, double error, double u_unsat, double int_max) {
    const bool saturated = std::abs(u_unsat) > TAU_MAX;
    if (!(saturated && sgn(error) == sgn(u_unsat))) {
        integral += Ts * error;
    }
    integral = std::clamp(integral, -int_max, int_max);
}

} // namespace

Control CascadeController::update(const Eigen::Vector4d& ref, const State& state) {
    const CascadeGains& g = gains_;

    const double ex_world = ref(0) - state(0);
    const double ey_world = ref(1) - state(1);
    const double ez = ref(2) - state(2);
    const double epsi = ref(3) - state(8);

    const double psi = state(8);
    const double ex_body = std::cos(psi) * ex_world + std::sin(psi) * ey_world;
    const double ey_body = -std::sin(psi) * ex_world + std::cos(psi) * ey_world;

    const double ex_sat = std::clamp(ex_body, -g.pos_sat, g.pos_sat);
    const double ey_sat = std::clamp(ey_body, -g.pos_sat, g.pos_sat);

    // Pitch forward accelerates +X; roll right accelerates -Y.
    double theta_des = g.gain_pos * ex_sat - g.gain_vel * state(3);
    double phi_des = -g.gain_pos * ey_sat + g.gain_vel * state(4);
    phi_des = std::clamp(phi_des, -g.angle_max, g.angle_max);
    theta_des = std::clamp(theta_des, -g.angle_max, g.angle_max);

    const double Kp_z = state(2) < g.takeoff_height ? g.Kp_z_takeoff : g.Kp_z;
    double F_cmd = HOVER_THRUST + Kp_z * ez - g.Kd_z * state(5);
    F_cmd = std::clamp(F_cmd, 0.0, 2.0 * HOVER_THRUST);

    const double ephi = phi_des - state(6);
    const double etheta = theta_des - state(7);

    const double tau_x_unsat = g.Kp_phi * ephi + g.Ki_phi * int_phi_ - g.Kd_phi * state(9);
    const double tau_y_unsat = g.Kp_theta * etheta + g.Ki_theta * int_theta_ - g.Kd_theta * state(10);
    const double tau_z_unsat = g.Kp_psi * epsi - g.Kd_psi * state(11);

    integrateWithAntiWindup(int_phi_, ephi, tau_x_unsat, g.int_max);
    integrateWithAntiWindup(int_theta_, etheta, tau_y_unsat, g.int_max);
    integrateWithAntiWindup(int_psi_, epsi, tau_z_unsat, g.int_max);

    const double tau_x = std::clamp(tau_x_unsat, -TAU_MAX, TAU_MAX);
    const double tau_y = std::clamp(tau_y_unsat, -TAU_MAX, TAU_MAX);
    const double tau_z = std::clamp(tau_z_unsat, -TAU_MAX, TAU_MAX);

    Control u = Quadcopter::mixerInverse(F_cmd, tau_x, tau_y, tau_z);
    return Quadcopter::saturate(u);
}

void CascadeController::reset() {
    int_phi_ = 0.0;
    int_theta_ = 0.0;
    int_psi_ = 0.0;
}

} // namespace quad
