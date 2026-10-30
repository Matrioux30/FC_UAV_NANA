#include "scenario.h"
#include <cmath>

namespace quad {

Scenario::Scenario(const ScenarioConfig& config) : config_(config) {}

Eigen::Vector4d Scenario::getReference(double t) const {
    Eigen::Vector4d ref = Eigen::Vector4d::Zero();

    switch (config_.type) {
        case ScenarioType::HOVER:
            ref << 0.0, 0.0, 1.0, 0.0;
            break;

        case ScenarioType::STEP:
            ref << 0.0, 0.0, (t < 1.0) ? 0.0 : 1.5, 0.0;
            break;

        case ScenarioType::TRAJECTORY: {
            const double R = 0.5;
            const double omega = 0.5;
            ref(0) = R * std::cos(omega * t);
            ref(1) = R * std::sin(omega * t);
            ref(2) = 1.0 + 0.2 * std::sin(t);
            ref(3) = 0.0;
            break;
        }

        case ScenarioType::TRAJECTORY_OKASHA: {
            // Okasha et al. (2022) Eq. 46-50. The paper uses NED (z down, z_ref = -1.5);
            // this model is z-up, so the altitude reference is +1.5 m.
            double x_int = 0.0;
            if (t >= 25.0 && t < 35.0) x_int = -0.5 * (t - 25.0);
            else if (t >= 35.0 && t < 55.0) x_int = -5.0;
            else if (t >= 55.0 && t < 65.0) x_int = -5.0 + 0.5 * (t - 55.0);

            double y_int = 0.0;
            if (t >= 10.0 && t < 20.0) y_int = -0.5 * (t - 10.0);
            else if (t >= 20.0 && t < 40.0) y_int = -5.0;
            else if (t >= 40.0 && t < 50.0) y_int = -5.0 + 0.5 * (t - 40.0);

            constexpr double PI = 3.14159265358979323846;
            if (t >= 10.0) {
                ref(0) = x_int + 2.0 * std::sin(0.1 * PI * t + 0.3);
                ref(1) = y_int + 2.0 * std::sin(0.1 * PI * t + 0.6);
                ref(2) = 1.5 + 0.1 * std::sin(0.02 * PI * t + 0.3);
            } else {
                ref(0) = x_int;
                ref(1) = y_int;
                ref(2) = 1.5;
            }
            ref(3) = 0.0;
            break;
        }
    }

    return ref;
}

ScenarioConfig makeHoverConfig() {
    ScenarioConfig config;
    config.type = ScenarioType::HOVER;
    config.duration = 50.0;
    config.name = "hover";
    config.x0 = State::Zero();
    config.x0(0) = 0.1;
    config.x0(1) = -0.05;
    config.x0(2) = 0.5;
    config.x0(6) = 0.05;
    config.x0(7) = -0.03;
    return config;
}

ScenarioConfig makeStepConfig() {
    ScenarioConfig config;
    config.type = ScenarioType::STEP;
    config.duration = 50.0;
    config.name = "step";
    config.x0 = State::Zero();
    return config;
}

ScenarioConfig makeTrajectoryConfig() {
    ScenarioConfig config;
    config.type = ScenarioType::TRAJECTORY;
    config.duration = 50.0;
    config.name = "trajectory";
    config.x0 = State::Zero();
    config.x0(2) = 1.0;
    return config;
}

ScenarioConfig makeTrajectoryOkashaConfig() {
    ScenarioConfig config;
    config.type = ScenarioType::TRAJECTORY_OKASHA;
    config.duration = 100.0;
    config.name = "trajectory_okasha";
    config.x0 = State::Zero();
    config.x0(2) = 1.0;
    return config;
}

} // namespace quad
