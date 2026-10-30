#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <Eigen/Dense>

namespace quad {

constexpr double MASS = 0.068;
constexpr double ARM_LENGTH = 0.062;
constexpr double GRAVITY = 9.81;
constexpr double Ix = 6.86e-5;
constexpr double Iy = 9.2e-5;
constexpr double Iz = 1.366e-4;
constexpr double kF = 0.01;
constexpr double kM = 7.8263e-4;
constexpr double Ts = 0.005;

constexpr double HOVER_THRUST = MASS * GRAVITY;
constexpr double MOTOR_DIST = ARM_LENGTH / std::sqrt(2.0);
constexpr double TAU_MAX = 0.02;
constexpr double OMEGA_MAX = 100.0;
constexpr double THETA_LIMIT = 1.45;

using State = Eigen::Matrix<double, 12, 1>;
using Control = Eigen::Matrix<double, 4, 1>;

class Quadcopter {
public:
    Quadcopter();
    explicit Quadcopter(const State& x0);

    void setState(const State& x);
    State getState() const;

    State derivative(const State& x, const Control& u) const;
    State step(const State& x, const Control& u, double dt) const;

    static Control mixerInverse(double F_total, double tau_x, double tau_y, double tau_z);
    static Control saturate(const Control& u);

private:
    State state_;
};

} // namespace quad

#endif
