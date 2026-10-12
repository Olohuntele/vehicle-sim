#include "vehicle_core.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle_sim {

VehicleCoreController::VehicleCoreController(const CollisionAvoidance::Config& caConfig,
                                             const ACCController::Config& accConfig)
    : collision_avoidance_(caConfig), acc_controller_(accConfig) {}

CoreOutput VehicleCoreController::evaluate(const CoreInput& input) {
    CoreOutput output;

    // 1. Validate inputs conservatively against NaN / negative velocities
    double v = std::max(0.0, input.ego_state.velocity);
    double a_brake = (input.max_braking > 0.0) ? input.max_braking : 6.0;
    double tau = std::max(0.0, input.system_delay);
    double margin = std::max(0.0, input.safety_margin);

    // 2. Dynamic speed-dependent stopping distance calculation:
    // d_stop = v * tau + (v^2) / (2 * a_brake) + margin
    double stopping_dist = (v * tau) + (v * v) / (2.0 * a_brake) + margin;
    output.estimated_stopping_distance = stopping_dist;

    // 3. Evaluate Collision Avoidance (AEB)
    double aebIntervention = collision_avoidance_.evaluate(input.ego_state, input.obstacle);
    output.aeb_triggered = (aebIntervention < 0.0);

    // 4. Bumper Clearance & Relative Motion
    bool has_obstacle = input.obstacle.detected && (input.obstacle.position < 900.0);
    double ego_front = input.ego_state.position + input.vehicle_length / 2.0;

    if (has_obstacle) {
        double obstacle_rear = input.obstacle.position - input.obstacle_length / 2.0;
        output.bumper_clearance = obstacle_rear - ego_front;
        output.relative_distance = input.obstacle.position - input.ego_state.position;
        if (output.relative_distance < 0.0) output.relative_distance = 0.0;

        output.relative_speed = v - input.obstacle.velocity; // positive = closing speed
        output.ttc = (output.relative_speed > 0.001) ? (output.relative_distance / output.relative_speed) : 99.9;
        output.collision_detected = (output.bumper_clearance <= 0.0);

        // Moving obstacle adjustment: if lead vehicle is moving, account for relative motion
        double effective_stopping_dist = stopping_dist;
        if (input.obstacle.velocity > 0.0 && output.relative_speed > 0.0) {
            effective_stopping_dist = (output.relative_speed * tau) + (output.relative_speed * output.relative_speed) / (2.0 * a_brake) + margin;
        }

        // 5. Arbitration: AEB > Planned Decel > ACC > Cruising
        if (output.aeb_triggered || output.bumper_clearance <= effective_stopping_dist) {
            output.control_input = -1.0; // Emergency braking override
            output.acc_active = false;
            output.braking_state = BrakingState::EMERGENCY_AEB;
            output.warning_level = WarningLevel::LEVEL_RED;
        } else if (output.bumper_clearance <= effective_stopping_dist * 1.5 || output.ttc < 3.0) {
            output.control_input = -0.5; // Planned deceleration
            output.acc_active = false;
            output.braking_state = BrakingState::PLANNED_DECEL;
            output.warning_level = WarningLevel::LEVEL_YELLOW;
        } else if (input.auto_drive) {
            output.acc_active = true;
            output.control_input = acc_controller_.computeControl(input.ego_state, input.obstacle);
            output.braking_state = BrakingState::CRUISING;
            output.warning_level = WarningLevel::LEVEL_BLUE;
        } else {
            output.acc_active = false;
            if (input.key_right_pressed) {
                output.control_input = 0.5;
            } else if (input.key_left_pressed) {
                output.control_input = -0.5;
            } else {
                output.control_input = input.manual_input != 0.0 ? input.manual_input : -0.1;
            }
            output.braking_state = BrakingState::COASTING;
            output.warning_level = WarningLevel::LEVEL_BLUE;
        }
    } else {
        // Clear road: Destination B stopping check
        output.bumper_clearance = input.destination_b - ego_front;
        output.relative_distance = input.destination_b - input.ego_state.position;
        output.relative_speed = v;
        output.ttc = 99.9;
        output.collision_detected = false;

        double dist_to_b = input.destination_b - ego_front;

        if (dist_to_b <= stopping_dist + 1.0 || input.ego_state.position >= input.destination_b) {
            output.control_input = -1.0; // Stop at destination
            output.acc_active = false;
            output.braking_state = BrakingState::STOPPING_AT_DESTINATION;
            output.warning_level = (dist_to_b <= stopping_dist) ? WarningLevel::LEVEL_YELLOW : WarningLevel::LEVEL_BLUE;
        } else if (input.auto_drive) {
            output.acc_active = true;
            if (v < 10.0) {
                output.control_input = 0.5; // Accelerate towards B
                output.braking_state = BrakingState::ACCELERATING;
                output.warning_level = WarningLevel::LEVEL_BLUE;
            } else {
                output.control_input = 0.0; // Cruise
                output.braking_state = BrakingState::CRUISING;
                output.warning_level = WarningLevel::LEVEL_BLUE;
            }
        } else {
            output.acc_active = false;
            if (input.key_right_pressed) {
                output.control_input = 0.5;
            } else if (input.key_left_pressed) {
                output.control_input = -0.5;
            } else {
                output.control_input = input.manual_input != 0.0 ? input.manual_input : -0.1;
            }
            output.braking_state = BrakingState::COASTING;
            output.warning_level = WarningLevel::LEVEL_BLUE;
        }
    }

    return output;
}

} // namespace vehicle_sim
