#ifndef SCENARIO_H
#define SCENARIO_H

#include "quadcopter.h"
#include <Eigen/Dense>
#include <string>

namespace quad {

enum class ScenarioType {
    HOVER,
    STEP,
    TRAJECTORY,
    TRAJECTORY_OKASHA
};

struct ScenarioConfig {
    ScenarioType type;
    double duration;
    State x0;
    std::string name;
};

class Scenario {
public:
    explicit Scenario(const ScenarioConfig& config);

    Eigen::Vector4d getReference(double t) const;
    double getDuration() const { return config_.duration; }
    const std::string& getName() const { return config_.name; }
    const State& getInitialState() const { return config_.x0; }

private:
    ScenarioConfig config_;
};

ScenarioConfig makeHoverConfig();
ScenarioConfig makeStepConfig();
ScenarioConfig makeTrajectoryConfig();
ScenarioConfig makeTrajectoryOkashaConfig();

} // namespace quad

#endif
