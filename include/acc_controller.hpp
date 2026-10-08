#pragma once

#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include <algorithm>

namespace vehicle_sim {

class ACCController {
public:
    struct Config {
        double target_speed = 10.0;    // m/s (cruising target speed)
        double time_gap = 1.5;         // seconds (desired time headway)
        double kp = 0.5;               // proportional gain for speed tracking
        double k_distance = 0.3;       // gain for distance error
    };

    ACCController(const Config& config) : config_(config) {}

    /**
     * @brief Computes control acceleration input for Adaptive Cruise Control.
     */
    double computeControl(const VehicleState& vehicle, const Obstacle& obstacle) {
        if (!obstacle.detected) {
            // Free flow cruising: track target speed
            double speed_error = config_.target_speed - vehicle.velocity;
            return std::clamp(config_.kp * speed_error, -1.0, 1.0);
        }

        double relative_distance = obstacle.position - vehicle.position;
        double desired_distance = vehicle.velocity * config_.time_gap + 5.0; // min 5m gap

        double distance_error = relative_distance - desired_distance;
        double relative_velocity = vehicle.velocity - obstacle.velocity;

        // ACC control law blending speed tracking and distance keeping
        double acceleration_command = config_.kp * (config_.target_speed - vehicle.velocity) + 
                                      config_.k_distance * (distance_error - relative_velocity);

        return std::clamp(acceleration_command / 3.5, -1.0, 1.0);
    }

private:
    Config config_;
};

} // namespace vehicle_sim
