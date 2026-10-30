#include "quadcopter.h"
#include "cascade_controller.h"
#include "scenario.h"
#include <iostream>
#include <iomanip>

using namespace quad;

int main() {
    ScenarioConfig config = makeHoverConfig();
    Quadcopter quad(config.x0);
    CascadeController controller;

    const int num_steps = static_cast<int>(config.duration / Ts);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t,x,y,z,phi,theta,psi,p,q,r,F_total,tau_x,tau_y,tau_z,w1,w2,w3,w4\n";

    for (int k = 0; k <= num_steps; ++k) {
        const double t = k * Ts;
        const Eigen::Vector4d ref(0.0, 0.0, 1.0, 0.0);
        const State state = quad.getState();

        const Control u = controller.update(ref, state);

        if (k % 10 == 0 || k < 30) {
            const double F_total = kF * (u(0)*u(0) + u(1)*u(1) + u(2)*u(2) + u(3)*u(3));
            const double tau_x = MOTOR_DIST * kF * (-u(0)*u(0) + u(1)*u(1) + u(2)*u(2) - u(3)*u(3));
            const double tau_y = MOTOR_DIST * kF * (-u(0)*u(0) - u(1)*u(1) + u(2)*u(2) + u(3)*u(3));
            const double tau_z = kM * (u(0)*u(0) - u(1)*u(1) + u(2)*u(2) - u(3)*u(3));
            std::cout << t << "," << state(0) << "," << state(1) << "," << state(2) << ","
                      << state(6) << "," << state(7) << "," << state(8) << ","
                      << state(9) << "," << state(10) << "," << state(11) << ","
                      << F_total << "," << tau_x << "," << tau_y << "," << tau_z << ","
                      << u(0) << "," << u(1) << "," << u(2) << "," << u(3) << "\n";
        }

        const State new_state = quad.step(state, u, Ts);
        quad.setState(new_state);

        if (std::isnan(new_state(0)) || std::isnan(new_state(2))) {
            std::cout << "DIVERGED at t=" << t << "\n";
            break;
        }
    }
    return 0;
}
