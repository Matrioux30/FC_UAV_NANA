#include "quadcopter.h"
#include <cmath>

namespace quad {

Quadcopter::Quadcopter() : state_(State::Zero()) {
    state_(8) = 0.0;
}

Quadcopter::Quadcopter(const State& x0) : state_(x0) {}

void Quadcopter::setState(const State& x) {
    state_ = x;
}

State Quadcopter::getState() const {
    return state_;
}

State Quadcopter::derivative(const State& x, const Control& u) const {
    State dx = State::Zero();

    const double phi = x(6);
    const double theta = x(7);
    const double psi = x(8);
    const double p = x(9);
    const double q = x(10);
    const double r = x(11);

    const double u_vel = x(3);
    const double v_vel = x(4);
    const double w_vel = x(5);

    const double F_total = kF * (u(0) * u(0) + u(1) * u(1) + u(2) * u(2) + u(3) * u(3));
    const double tau_x = MOTOR_DIST * kF * (-u(0) * u(0) + u(1) * u(1) + u(2) * u(2) - u(3) * u(3));
    const double tau_y = MOTOR_DIST * kF * (-u(0) * u(0) - u(1) * u(1) + u(2) * u(2) + u(3) * u(3));
    const double tau_z = kM * (u(0) * u(0) - u(1) * u(1) + u(2) * u(2) - u(3) * u(3));

    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);
    double theta_clamped = theta;
    if (theta_clamped > THETA_LIMIT) theta_clamped = THETA_LIMIT;
    else if (theta_clamped < -THETA_LIMIT) theta_clamped = -THETA_LIMIT;
    const double cos_theta = std::cos(theta_clamped);
    const double sin_theta = std::sin(theta_clamped);
    const double cos_psi = std::cos(psi);
    const double sin_psi = std::sin(psi);
    const double tan_theta = std::tan(theta_clamped);

    dx(0) = cos_theta * cos_psi * u_vel +
            (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * v_vel +
            (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * w_vel;

    dx(1) = cos_theta * sin_psi * u_vel +
            (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * v_vel +
            (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * w_vel;

    dx(2) = -sin_theta * u_vel + sin_phi * cos_theta * v_vel + cos_phi * cos_theta * w_vel;

    dx(3) = r * v_vel - q * w_vel + GRAVITY * sin_theta;

    dx(4) = p * w_vel - r * u_vel - GRAVITY * cos_theta * sin_phi;

    dx(5) = q * u_vel - p * v_vel - GRAVITY * cos_theta * cos_phi + F_total / MASS;

    dx(6) = p + sin_phi * tan_theta * q + cos_phi * tan_theta * r;

    dx(7) = cos_phi * q - sin_phi * r;

    dx(8) = sin_phi / cos_theta * q + cos_phi / cos_theta * r;

    dx(9) = (q * r * (Iy - Iz) + tau_x) / Ix;

    dx(10) = (p * r * (Iz - Ix) + tau_y) / Iy;

    dx(11) = (p * q * (Ix - Iy) + tau_z) / Iz;

    return dx;
}

State Quadcopter::step(const State& x, const Control& u, double dt) const {
    return x + dt * derivative(x, u);
}

Control Quadcopter::mixerInverse(double F_total, double tau_x, double tau_y, double tau_z) {
    const double d = MOTOR_DIST;
    const double c = kM / kF;

    Eigen::Matrix4d M;
    M << 1,  1,  1,  1,
        -d,  d,  d, -d,
        -d, -d,  d,  d,
         c, -c,  c, -c;

    Eigen::Vector4d forces;
    forces << F_total / kF, tau_x / kF, tau_y / kF, tau_z / kF;

    Eigen::Vector4d omega_sq = M.inverse() * forces;

    Control u;
    for (int i = 0; i < 4; ++i) {
        double val = omega_sq(i);
        u(i) = val < 0.0 ? 0.0 : std::sqrt(val);
    }

    return u;
}

Control Quadcopter::saturate(const Control& u) {
    Control u_sat;
    for (int i = 0; i < 4; ++i) {
        if (u(i) > OMEGA_MAX) u_sat(i) = OMEGA_MAX;
        else if (u(i) < 0.0) u_sat(i) = 0.0;
        else u_sat(i) = u(i);
    }
    return u_sat;
}

} // namespace quad
