#include "pure_pursuit.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace vehicle_sim {

PurePursuitController::PurePursuitController(const PurePursuitConfig& config)
    : config_(config), current_target_index_(0) {}

void PurePursuitController::setConfig(const PurePursuitConfig& config) {
    config_ = config;
}

const PurePursuitConfig& PurePursuitController::getConfig() const {
    return config_;
}

void PurePursuitController::reset() {
    current_target_index_ = 0;
}

bool PurePursuitController::validateInputs(const Pose2D& pose, const std::vector<Waypoint>& waypoints) const {
    // Check pose finiteness
    if (!std::isfinite(pose.x) || !std::isfinite(pose.z) ||
        !std::isfinite(pose.heading) || !std::isfinite(pose.velocity)) {
        return false;
    }

    // Check configuration parameters
    if (!std::isfinite(config_.wheelbase) || config_.wheelbase <= 0.0 ||
        !std::isfinite(config_.lookahead_distance) || config_.lookahead_distance <= 0.0 ||
        !std::isfinite(config_.max_steering_angle) || config_.max_steering_angle <= 0.0 ||
        !std::isfinite(config_.arrival_tolerance) || config_.arrival_tolerance < 0.0) {
        return false;
    }

    // Check waypoints finiteness
    for (const auto& wp : waypoints) {
        if (!std::isfinite(wp.x) || !std::isfinite(wp.z)) {
            return false;
        }
    }

    return true;
}

double PurePursuitController::computeCrossTrackError(const Pose2D& pose, const std::vector<Waypoint>& waypoints, size_t& nearest_seg_idx) const {
    if (waypoints.empty()) return 0.0;
    if (waypoints.size() == 1) {
        return std::hypot(waypoints[0].x - pose.x, waypoints[0].z - pose.z);
    }

    double min_dist_sq = 1e18;
    nearest_seg_idx = 0;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        double x1 = waypoints[i].x;
        double z1 = waypoints[i].z;
        double x2 = waypoints[i + 1].x;
        double z2 = waypoints[i + 1].z;

        double dx = x2 - x1;
        double dz = z2 - z1;
        double len_sq = dx * dx + dz * dz;

        // Handle degenerate/duplicate waypoints (zero segment length)
        if (len_sq < 1e-6) {
            continue;
        }

        double t = ((pose.x - x1) * dx + (pose.z - z1) * dz) / len_sq;
        t = std::clamp(t, 0.0, 1.0);

        double proj_x = x1 + t * dx;
        double proj_z = z1 + t * dz;

        double dist_sq = (pose.x - proj_x) * (pose.x - proj_x) + (pose.z - proj_z) * (pose.z - proj_z);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_seg_idx = i;
        }
    }

    if (min_dist_sq > 1e16) {
        // Fallback if all segments were degenerate
        min_dist_sq = (waypoints[0].x - pose.x) * (waypoints[0].x - pose.x) + 
                      (waypoints[0].z - pose.z) * (waypoints[0].z - pose.z);
    }

    return std::sqrt(min_dist_sq);
}

SteeringOutput PurePursuitController::computeSteering(const Pose2D& ego_pose, const std::vector<Waypoint>& waypoints) {
    SteeringOutput output;

    // 1. Validate inputs
    if (!validateInputs(ego_pose, waypoints)) {
        output.valid = false;
        output.fault_detected = true;
        output.fault_reason = "Invalid or non-finite inputs detected in Pose2D, config, or waypoints";
        output.steering_command = 0.0;
        output.steering_angle = 0.0;
        return output;
    }

    // 2. Handle empty route
    if (waypoints.empty()) {
        output.valid = true;
        output.route_completed = true;
        output.steering_command = 0.0;
        output.steering_angle = 0.0;
        return output;
    }

    // 3. Final arrival check (check distance to final waypoint)
    double dist_to_final = std::hypot(waypoints.back().x - ego_pose.x,
                                      waypoints.back().z - ego_pose.z);
    if (dist_to_final <= config_.arrival_tolerance) {
        output.valid = true;
        output.route_completed = true;
        output.steering_command = 0.0;
        output.steering_angle = 0.0;
        output.target_waypoint_index = waypoints.size() - 1;
        return output;
    }

    // 4. Compute cross-track error
    size_t nearest_seg = 0;
    output.cross_track_error = computeCrossTrackError(ego_pose, waypoints, nearest_seg);

    // 5. Waypoint progression / Target selection
    double L_d = std::max(config_.lookahead_distance, config_.min_lookahead_distance);
    size_t target_idx = current_target_index_;
    
    if (target_idx >= waypoints.size()) {
        target_idx = waypoints.size() - 1;
    }

    bool found_target = false;
    for (size_t i = target_idx; i < waypoints.size(); ++i) {
        // Skip duplicate or degenerate waypoints that are identical to previous
        if (i > 0 && std::hypot(waypoints[i].x - waypoints[i-1].x, waypoints[i].z - waypoints[i-1].z) < 1e-4) {
            continue;
        }
        double dist = std::hypot(waypoints[i].x - ego_pose.x, waypoints[i].z - ego_pose.z);
        if (dist >= L_d) {
            target_idx = i;
            found_target = true;
            break;
        }
    }

    if (!found_target) {
        target_idx = waypoints.size() - 1;
    }

    current_target_index_ = target_idx;
    output.target_waypoint_index = current_target_index_;

    Waypoint target_wp = waypoints[current_target_index_];

    // 6. Transform target point into vehicle local coordinate frame
    // Coordinate Conventions:
    // - Z_veh: forward (along heading theta)
    // - X_veh: lateral (positive = LEFT, negative = RIGHT)
    double dx = target_wp.x - ego_pose.x;
    double dz = target_wp.z - ego_pose.z;
    double theta = ego_pose.heading;

    double z_veh = dx * std::sin(theta) + dz * std::cos(theta);
    double x_veh = -(dx * std::cos(theta) - dz * std::sin(theta)); // Positive = Left

    // If target is behind vehicle (z_veh < 0) and we are not at the final waypoint, advance target index
    if (z_veh < 0.0 && current_target_index_ < waypoints.size() - 1) {
        current_target_index_++;
        target_wp = waypoints[current_target_index_];
        output.target_waypoint_index = current_target_index_;
        dx = target_wp.x - ego_pose.x;
        dz = target_wp.z - ego_pose.z;
        z_veh = dx * std::sin(theta) + dz * std::cos(theta);
        x_veh = -(dx * std::cos(theta) - dz * std::sin(theta));
    }

    // 7. Pure Pursuit Curvature and Steering Angle Calculation
    // Curvature kappa = 2 * x_veh / L_d^2
    // Steering angle delta = atan(kappa * wheelbase)
    double ld_sq = std::max(L_d * L_d, 1e-4);
    double curvature = (2.0 * x_veh) / ld_sq;

    double steering_angle = std::atan(curvature * config_.wheelbase);
    steering_angle = std::clamp(steering_angle, -config_.max_steering_angle, config_.max_steering_angle);

    // 8. Output normalization [-1.0, 1.0] (+1.0 = full left, -1.0 = full right)
    double normalized_cmd = 0.0;
    if (config_.max_steering_angle > 1e-6) {
        normalized_cmd = steering_angle / config_.max_steering_angle;
    }
    normalized_cmd = std::clamp(normalized_cmd, -1.0, 1.0);

    if (!std::isfinite(steering_angle) || !std::isfinite(normalized_cmd)) {
        output.valid = false;
        output.fault_detected = true;
        output.fault_reason = "Non-finite steering angle or command computed";
        output.steering_command = 0.0;
        output.steering_angle = 0.0;
        return output;
    }

    output.valid = true;
    output.fault_detected = false;
    output.steering_angle = steering_angle;
    output.steering_command = normalized_cmd;

    return output;
}

} // namespace vehicle_sim
