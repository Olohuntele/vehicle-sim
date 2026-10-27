#include "easy_drive_bridge.h"
#include "vehicle_core.hpp"
#include "pure_pursuit.hpp"
#include "actuator_commands.hpp"
#include <cmath>
#include <vector>

namespace godot {

EasyDriveBridge::EasyDriveBridge() {
    vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -6.0};
    vehicle_sim::ACCController::Config accConfig{10.0, 1.5, 0.5, 0.3};
    controller_ = new vehicle_sim::VehicleCoreController(caConfig, accConfig);
    pure_pursuit_controller_ = new vehicle_sim::PurePursuitController();
}

EasyDriveBridge::~EasyDriveBridge() {
    if (controller_) {
        delete controller_;
        controller_ = nullptr;
    }
    if (pure_pursuit_controller_) {
        delete pure_pursuit_controller_;
        pure_pursuit_controller_ = nullptr;
    }
}

String EasyDriveBridge::get_bridge_status() const {
    return "C++ GDExtension connected with Pure Pursuit Steering";
}

Dictionary EasyDriveBridge::evaluate_shadow_step(
    double ego_pos, double ego_speed, double ego_accel,
    bool obstacle_detected, double obstacle_pos, double obstacle_speed,
    bool auto_drive, double manual_input, double destination_b
) {
    Dictionary result;
    if (!controller_) return result;

    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_pos};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput output = controller_->evaluate(input);

    result["control_input"] = output.control_input;
    result["aeb_triggered"] = output.aeb_triggered;
    result["acc_active"] = output.acc_active;
    result["relative_distance"] = output.relative_distance;
    result["relative_speed"] = output.relative_speed;
    result["ttc"] = output.ttc;
    result["bumper_clearance"] = output.bumper_clearance;
    result["estimated_stopping_distance"] = output.estimated_stopping_distance;
    result["warning_level"] = static_cast<int>(output.warning_level);
    result["braking_state"] = static_cast<int>(output.braking_state);
    result["collision_detected"] = output.collision_detected;

    return result;
}

Dictionary EasyDriveBridge::evaluate_actuator_step(
    double ego_pos, double ego_speed, double ego_accel,
    bool obstacle_detected, double obstacle_pos, double obstacle_speed,
    bool auto_drive, double manual_input, double destination_b
) {
    Dictionary result;
    if (!controller_) return result;

    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_pos};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput output = controller_->evaluate(input);
    vehicle_sim::ActuatorAdapter adapter;
    vehicle_sim::ActuatorCommand cmd = adapter.convert(output);

    // Telemetry fields
    result["control_input"] = output.control_input;
    result["aeb_triggered"] = output.aeb_triggered;
    result["acc_active"] = output.acc_active;
    result["relative_distance"] = output.relative_distance;
    result["relative_speed"] = output.relative_speed;
    result["ttc"] = output.ttc;
    result["bumper_clearance"] = output.bumper_clearance;
    result["estimated_stopping_distance"] = output.estimated_stopping_distance;
    result["warning_level"] = static_cast<int>(output.warning_level);
    result["braking_state"] = static_cast<int>(output.braking_state);
    result["collision_detected"] = output.collision_detected;

    // Actuator command fields
    result["throttle_demand"] = cmd.throttle_demand;
    result["brake_demand"] = cmd.brake_demand;
    result["steering_demand"] = cmd.steering_demand;
    result["emergency_braking"] = cmd.emergency_braking;
    result["valid"] = cmd.valid;
    result["fault_detected"] = cmd.fault_detected;
    result["fault_reason"] = String(cmd.fault_reason.c_str());

    return result;
}

Dictionary EasyDriveBridge::evaluate_path_step(
    double ego_x, double ego_z, double ego_heading, double ego_speed, double ego_accel,
    const Array& waypoints_array,
    bool obstacle_detected, double obstacle_pos, double obstacle_speed,
    bool auto_drive, double manual_input, double destination_b
) {
    Dictionary result;
    if (!controller_ || !pure_pursuit_controller_) return result;

    // 1. Parse waypoints from Godot Array
    // Coordinate conventions: +Z forward, +X right. Godot Vector2 (x, y) or Vector3 (x, z) or Dictionary (x, z).
    std::vector<vehicle_sim::Waypoint> waypoints;
    bool waypoints_valid = true;
    std::string waypoints_fault_reason = "";

    for (int i = 0; i < waypoints_array.size(); ++i) {
        Variant v = waypoints_array[i];
        vehicle_sim::Waypoint wp;
        if (v.get_type() == Variant::VECTOR2) {
            Vector2 vec = v;
            wp.x = vec.x;
            wp.z = vec.y;
        } else if (v.get_type() == Variant::VECTOR3) {
            Vector3 vec = v;
            wp.x = vec.x;
            wp.z = vec.z;
        } else if (v.get_type() == Variant::DICTIONARY) {
            Dictionary dict = v;
            if (dict.has("x") && dict.has("z")) {
                wp.x = dict["x"];
                wp.z = dict["z"];
            } else if (dict.has("x") && dict.has("y")) {
                wp.x = dict["x"];
                wp.z = dict["y"];
            } else {
                waypoints_valid = false;
                waypoints_fault_reason = "Waypoint dictionary missing 'x' and 'z' keys";
                break;
            }
        } else {
            waypoints_valid = false;
            waypoints_fault_reason = "Unsupported waypoint element type in array";
            break;
        }

        if (!std::isfinite(wp.x) || !std::isfinite(wp.z)) {
            waypoints_valid = false;
            waypoints_fault_reason = "Non-finite waypoint coordinate detected";
            break;
        }
        waypoints.push_back(wp);
    }

    // 2. Evaluate Pure Pursuit Steering
    vehicle_sim::Pose2D ego_pose{ego_x, ego_z, ego_heading, ego_speed};
    vehicle_sim::SteeringOutput steering_output;

    if (!waypoints_valid) {
        steering_output.valid = false;
        steering_output.fault_detected = true;
        steering_output.fault_reason = waypoints_fault_reason;
        steering_output.steering_command = 0.0;
        steering_output.steering_angle = 0.0;
    } else {
        steering_output = pure_pursuit_controller_->computeSteering(ego_pose, waypoints);
    }

    // 3. Evaluate Longitudinal Core Controller
    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_z};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput core_output = controller_->evaluate(input);

    // 4. Convert via ActuatorAdapter (combining longitudinal and lateral steering validation)
    vehicle_sim::ActuatorAdapter adapter;
    vehicle_sim::ActuatorCommand cmd = adapter.convert(core_output, steering_output);

    // 5. Populate Result Dictionary
    // Longitudinal telemetry
    result["control_input"] = core_output.control_input;
    result["aeb_triggered"] = core_output.aeb_triggered;
    result["acc_active"] = core_output.acc_active;
    result["relative_distance"] = core_output.relative_distance;
    result["relative_speed"] = core_output.relative_speed;
    result["ttc"] = core_output.ttc;
    result["bumper_clearance"] = core_output.bumper_clearance;
    result["estimated_stopping_distance"] = core_output.estimated_stopping_distance;
    result["warning_level"] = static_cast<int>(core_output.warning_level);
    result["braking_state"] = static_cast<int>(core_output.braking_state);
    result["collision_detected"] = core_output.collision_detected;

    // Actuator commands & lateral steering demand
    result["throttle_demand"] = cmd.throttle_demand;
    result["brake_demand"] = cmd.brake_demand;
    result["steering_demand"] = cmd.steering_demand;
    result["emergency_braking"] = cmd.emergency_braking;
    result["valid"] = cmd.valid;
    result["fault_detected"] = cmd.fault_detected;
    result["fault_reason"] = String(cmd.fault_reason.c_str());

    // Route tracking telemetry
    result["target_waypoint_index"] = static_cast<int64_t>(steering_output.target_waypoint_index);
    result["cross_track_error"] = steering_output.cross_track_error;
    result["route_completed"] = steering_output.route_completed;
    result["steering_angle"] = steering_output.steering_angle;

    return result;
}

void EasyDriveBridge::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_bridge_status"), &EasyDriveBridge::get_bridge_status);
    ClassDB::bind_method(D_METHOD("evaluate_shadow_step",
        "ego_pos", "ego_speed", "ego_accel",
        "obstacle_detected", "obstacle_pos", "obstacle_speed",
        "auto_drive", "manual_input", "destination_b"),
        &EasyDriveBridge::evaluate_shadow_step);
    ClassDB::bind_method(D_METHOD("evaluate_actuator_step",
        "ego_pos", "ego_speed", "ego_accel",
        "obstacle_detected", "obstacle_pos", "obstacle_speed",
        "auto_drive", "manual_input", "destination_b"),
        &EasyDriveBridge::evaluate_actuator_step);
    ClassDB::bind_method(D_METHOD("evaluate_path_step",
        "ego_x", "ego_z", "ego_heading", "ego_speed", "ego_accel",
        "waypoints_array",
        "obstacle_detected", "obstacle_pos", "obstacle_speed",
        "auto_drive", "manual_input", "destination_b"),
        &EasyDriveBridge::evaluate_path_step);
}

} // namespace godot
