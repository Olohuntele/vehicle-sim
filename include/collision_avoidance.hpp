#pragma once

#include <iostream>
#include <string>

namespace vehicle_sim {

struct VehicleState {
    double velocity = 0.0;     // m/s
    double acceleration = 0.0; // m/s^2
    double position = 0.0;     // m
};

struct Obstacle {
    double position = 0.0;     // m
    double velocity = 0.0;     // m/s
    bool detected = false;
};

class CollisionAvoidance {
public:
    struct Config {
        double ttc_threshold = 2.0;       // Seconds
        double min_safe_distance = 5.0;  // Meters
        double max_braking = -8.0;      // m/s^2
    };

    CollisionAvoidance(const Config& config) : config_(config) {}

    /**
     * @brief Evaluates if braking is required.
     * @return Required acceleration (negative for braking, 0 for no intervention).
     */
    double evaluate(const VehicleState& vehicle, const Obstacle& obstacle) {
        if (!obstacle.detected) {
            return 0.0;
        }

        double relative_distance = obstacle.position - vehicle.position;
        double relative_velocity = vehicle.velocity - obstacle.velocity;

        // If obstacle is behind or moving away, no action needed
        if (relative_distance <= 0 || relative_velocity <= 0) {
            return 0.0;
        }

        double ttc = relative_distance / relative_velocity;

        if (ttc < config_.ttc_threshold || relative_distance < config_.min_safe_distance) {
            return config_.max_braking;
        }

        return 0.0;
    }

private:
    Config config_;
};

} // namespace vehicle_sim
