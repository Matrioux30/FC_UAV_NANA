#ifndef QUADCOPTER_MODEL_H
#define QUADCOPTER_MODEL_H

#include <cmath>
#include <array>
#include <algorithm>

namespace quad {

// Physical parameters of Parrot Mambo (Okasha et al., 2022)
constexpr double MASS = 0.068;
constexpr double ARM_LENGTH = 0.062;
constexpr double GRAVITY = 9.81;
constexpr double IX = 6.86e-5;
constexpr double IY = 9.2e-5;
constexpr double IZ = 1.366e-4;
constexpr double KF = 0.01;
constexpr double KM = 7.8263e-4;
constexpr double HOVER_THRUST = MASS * GRAVITY; // 0.66708 N
constexpr double MOTOR_DIST = ARM_LENGTH / 1.4142135623730951; // ARM_LENGTH / sqrt(2)
constexpr double THETA_LIMIT = 1.45;

struct QuadState {
    // 0:x, 1:y, 2:z, 3:u, 4:v, 5:w, 6:phi, 7:theta, 8:psi, 9:p, 10:q, 11:r
    std::array<double, 12> val{};
    
    double& operator[](size_t i) { return val[i]; }
    const double& operator[](size_t i) const { return val[i]; }
};

struct QuadControl {
    double F_total = HOVER_THRUST;
    double tau_x = 0.0;
    double tau_y = 0.0;
    double tau_z = 0.0;
};

// Nonlinear dynamics step (Newton-Euler, same as verified in MATLAB & quadcopter_pid)
inline QuadState quadcopter_step(const QuadState& x, const QuadControl& u_cmd, double dt = 0.005) {
    const double phi = x[6];
    const double theta = x[7];
    const double psi = x[8];
    const double p = x[9];
    const double q = x[10];
    const double r = x[11];

    const double u_vel = x[3];
    const double v_vel = x[4];
    const double w_vel = x[5];

    const double F_total = u_cmd.F_total;
    const double tau_x = u_cmd.tau_x;
    const double tau_y = u_cmd.tau_y;
    const double tau_z = u_cmd.tau_z;

    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);
    double theta_clamped = std::clamp(theta, -THETA_LIMIT, THETA_LIMIT);
    const double cos_theta = std::cos(theta_clamped);
    const double sin_theta = std::sin(theta_clamped);
    const double cos_psi = std::cos(psi);
    const double sin_psi = std::sin(psi);
    const double tan_theta = std::tan(theta_clamped);

    std::array<double, 12> dx{};

    dx[0] = cos_theta * cos_psi * u_vel +
            (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) * v_vel +
            (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * w_vel;

    dx[1] = cos_theta * sin_psi * u_vel +
            (sin_phi * sin_theta * sin_psi + cos_phi * cos_psi) * v_vel +
            (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * w_vel;

    dx[2] = -sin_theta * u_vel + sin_phi * cos_theta * v_vel + cos_phi * cos_theta * w_vel;

    dx[3] = r * v_vel - q * w_vel + GRAVITY * sin_theta;
    dx[4] = p * w_vel - r * u_vel - GRAVITY * cos_theta * sin_phi;
    dx[5] = q * u_vel - p * v_vel - GRAVITY * cos_theta * cos_phi + F_total / MASS;

    dx[6] = p + sin_phi * tan_theta * q + cos_phi * tan_theta * r;
    dx[7] = cos_phi * q - sin_phi * r;
    dx[8] = sin_phi / cos_theta * q + cos_phi / cos_theta * r;

    dx[9] = (q * r * (IY - IZ) + tau_x) / IX;
    dx[10] = (p * r * (IZ - IX) + tau_y) / IY;
    dx[11] = (p * q * (IX - IY) + tau_z) / IZ;

    QuadState x_next;
    for (int i = 0; i < 12; ++i) {
        x_next[i] = x[i] + dt * dx[i];
    }
    return x_next;
}

} // namespace quad

#endif // QUADCOPTER_MODEL_H
