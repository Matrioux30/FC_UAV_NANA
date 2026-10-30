#ifndef SCENARIOS_H
#define SCENARIOS_H

#include "mpc_constants.h"
#include "quadcopter_model.h"
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace quad {

enum class ScenarioType { HOVER, STEP, TRAJECTORY };

// Shared by golden model, empirical-iteration study and profiling so all results describe the same runs.
struct Scenario {
    std::string name;
    ScenarioType type;
    double duration;
    QuadState x0;

    int num_steps() const { return static_cast<int>(std::lround(duration / TS)); }

    // [x_ref, y_ref, z_ref, psi_ref] at time t (same references as quadcopter_pid).
    std::array<double, NREF> reference(double t) const {
        switch (type) {
            case ScenarioType::HOVER: return {0.0, 0.0, 1.0, 0.0};
            case ScenarioType::STEP: return {0.0, 0.0, t < 1.0 ? 0.0 : 1.5, 0.0};
            case ScenarioType::TRAJECTORY:
            default: return {0.5 * std::cos(0.5 * t), 0.5 * std::sin(0.5 * t), 1.0 + 0.2 * std::sin(t), 0.0};
        }
    }

    // Reference over the prediction horizon, layout r[p * NREF + c].
    void horizon(double t, double r[NR]) const {
        for (int p = 0; p < NP; ++p) {
            const auto v = reference(t + p * TS);
            for (int c = 0; c < NREF; ++c) r[p * NREF + c] = v[c];
        }
    }
};

inline std::vector<Scenario> golden_scenarios() {
    QuadState hover{};
    hover[0] = 0.1; hover[1] = -0.05; hover[2] = 0.5; hover[6] = 0.05; hover[7] = -0.03;
    QuadState step{};
    QuadState traj{};
    traj[0] = 0.5; traj[2] = 1.0;
    return {
        {"hover", ScenarioType::HOVER, 5.0, hover},
        {"step", ScenarioType::STEP, 5.0, step},
        {"trajectory", ScenarioType::TRAJECTORY, 10.0, traj},
    };
}

} // namespace quad

#endif // SCENARIOS_H
