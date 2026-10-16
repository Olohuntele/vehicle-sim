#include "easy_drive_bridge.h"
#include "vehicle_core.hpp"

namespace godot {

EasyDriveBridge::EasyDriveBridge() {
    vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -6.0};
    vehicle_sim::ACCController::Config accConfig{10.0, 1.5, 0.5, 0.3};
    controller_ = new vehicle_sim::VehicleCoreController(caConfig, accConfig);
}

EasyDriveBridge::~EasyDriveBridge() {
    if (controller_) {
        delete controller_;
        controller_ = nullptr;
    }
}

String EasyDriveBridge::get_bridge_status() const {
    return "C++ GDExtension connected";
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

void EasyDriveBridge::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_bridge_status"), &EasyDriveBridge::get_bridge_status);
    ClassDB::bind_method(D_METHOD("evaluate_shadow_step",
        "ego_pos", "ego_speed", "ego_accel",
        "obstacle_detected", "obstacle_pos", "obstacle_speed",
        "auto_drive", "manual_input", "destination_b"),
        &EasyDriveBridge::evaluate_shadow_step);
}

} // namespace godot
