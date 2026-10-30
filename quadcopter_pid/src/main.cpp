#include "quadcopter.h"
#include "cascade_controller.h"
#include "scenario.h"
#include "csv_writer.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace quad;

int main(int argc, char* argv[]) {
    std::string scenario_name = "hover";
    if (argc > 1) {
        scenario_name = argv[1];
    }

    ScenarioConfig config;
    if (scenario_name == "hover") {
        config = makeHoverConfig();
    } else if (scenario_name == "step") {
        config = makeStepConfig();
    } else if (scenario_name == "trajectory") {
        config = makeTrajectoryConfig();
    } else if (scenario_name == "trajectory_okasha") {
        config = makeTrajectoryOkashaConfig();
    } else {
        std::cerr << "Unknown scenario: " << scenario_name << "\n";
        std::cerr << "Usage: quadcopter_pid [hover|step|trajectory|trajectory_okasha]\n";
        return 1;
    }

    Scenario scenario(config);
    Quadcopter quad(config.x0);
    CascadeController controller;

    std::string csv_name = "comparison_plots/" + config.name + ".csv";
    CsvWriter writer(csv_name);
    writer.writeHeader();

    const double duration = scenario.getDuration();
    const int num_steps = static_cast<int>(std::lround(duration / Ts));

    for (int k = 0; k <= num_steps; ++k) {
        const double t = k * Ts;

        const Eigen::Vector4d ref = scenario.getReference(t);
        const State state = quad.getState();

        LogRow row;
        row.time = t;
        row.x_ref = ref(0);
        row.y_ref = ref(1);
        row.z_ref = ref(2);
        row.psi_ref = ref(3);
        row.x = state(0);
        row.y = state(1);
        row.z = state(2);
        row.phi = state(6);
        row.theta = state(7);
        row.psi = state(8);
        row.p = state(9);
        row.q = state(10);
        row.r = state(11);
        row.ex = ref(0) - state(0);
        row.ey = ref(1) - state(1);
        row.ez = ref(2) - state(2);

        const Control u = controller.update(ref, state);

        const State new_state = quad.step(state, u, Ts);
        quad.setState(new_state);

        const double F_total = kF * (u(0) * u(0) + u(1) * u(1) + u(2) * u(2) + u(3) * u(3));
        const double tau_x = MOTOR_DIST * kF * (-u(0) * u(0) + u(1) * u(1) + u(2) * u(2) - u(3) * u(3));
        const double tau_y = MOTOR_DIST * kF * (-u(0) * u(0) - u(1) * u(1) + u(2) * u(2) + u(3) * u(3));
        const double tau_z = kM * (u(0) * u(0) - u(1) * u(1) + u(2) * u(2) - u(3) * u(3));

        row.F_total = F_total;
        row.tau_x = tau_x;
        row.tau_y = tau_y;
        row.tau_z = tau_z;
        row.omega1 = u(0);
        row.omega2 = u(1);
        row.omega3 = u(2);
        row.omega4 = u(3);

        writer.writeRow(row);
    }

    writer.close();
    std::cout << "Scenario '" << config.name << "' completed. Results saved to " << csv_name << "\n";

    return 0;
}
