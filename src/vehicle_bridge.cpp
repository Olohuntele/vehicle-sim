#include "vehicle_core.hpp"
#include "pure_pursuit.hpp"
#include "actuator_commands.hpp"
#include <vector>

extern "C" {

struct ControllerContext {
    vehicle_sim::VehicleCoreController controller;
    vehicle_sim::PurePursuitController pure_pursuit;
    ControllerContext(const vehicle_sim::CollisionAvoidance::Config& ca,
                      const vehicle_sim::ACCController::Config& acc)
        : controller(ca, acc), pure_pursuit() {}
};

__declspec(dllexport) void* create_controller(double ttc_threshold, double min_safe_dist, double max_braking,
                                              double target_speed, double time_gap) {
    vehicle_sim::CollisionAvoidance::Config caConfig{ttc_threshold, min_safe_dist, -std::abs(max_braking)};
    vehicle_sim::ACCController::Config accConfig{target_speed, time_gap, 0.5, 0.3};
    return new ControllerContext(caConfig, accConfig);
}

__declspec(dllexport) void destroy_controller(void* handle) {
    if (handle) {
        delete static_cast<ControllerContext*>(handle);
    }
}

__declspec(dllexport) void evaluate_step(void* handle,
                                       double ego_pos, double ego_speed, double ego_accel,
                                       bool obstacle_detected, double obstacle_pos, double obstacle_speed,
                                       bool auto_drive, double manual_input, double destination_b,
                                       double* out_control_input, bool* out_aeb, bool* out_acc_active,
                                       double* out_bumper_clearance, double* out_stopping_dist, int* out_warning_level) {
    if (!handle) return;
    auto* ctx = static_cast<ControllerContext*>(handle);

    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_pos};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput output = ctx->controller.evaluate(input);

    if (out_control_input) *out_control_input = output.control_input;
    if (out_aeb) *out_aeb = output.aeb_triggered;
    if (out_acc_active) *out_acc_active = output.acc_active;
    if (out_bumper_clearance) *out_bumper_clearance = output.bumper_clearance;
    if (out_stopping_dist) *out_stopping_dist = output.estimated_stopping_distance;
    if (out_warning_level) *out_warning_level = static_cast<int>(output.warning_level);
}

__declspec(dllexport) void evaluate_actuator_step(void* handle,
                                                double ego_pos, double ego_speed, double ego_accel,
                                                bool obstacle_detected, double obstacle_pos, double obstacle_speed,
                                                bool auto_drive, double manual_input, double destination_b,
                                                double* out_throttle, double* out_brake, bool* out_emergency,
                                                bool* out_valid, bool* out_fault) {
    if (!handle) return;
    auto* ctx = static_cast<ControllerContext*>(handle);

    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_pos};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput output = ctx->controller.evaluate(input);
    vehicle_sim::ActuatorAdapter adapter;
    vehicle_sim::ActuatorCommand cmd = adapter.convert(output);

    if (out_throttle) *out_throttle = cmd.throttle_demand;
    if (out_brake) *out_brake = cmd.brake_demand;
    if (out_emergency) *out_emergency = cmd.emergency_braking;
    if (out_valid) *out_valid = cmd.valid;
    if (out_fault) *out_fault = cmd.fault_detected;
}

__declspec(dllexport) void evaluate_path_step(void* handle,
                                              double ego_x, double ego_z, double ego_heading, double ego_speed, double ego_accel,
                                              const double* wp_x, const double* wp_z, int wp_count,
                                              bool obstacle_detected, double obstacle_pos, double obstacle_speed,
                                              bool auto_drive, double manual_input, double destination_b,
                                              double* out_throttle, double* out_brake, double* out_steering,
                                              bool* out_emergency, bool* out_valid, bool* out_fault,
                                              bool* out_route_completed, double* out_cross_track_error) {
    if (!handle) return;
    auto* ctx = static_cast<ControllerContext*>(handle);

    std::vector<vehicle_sim::Waypoint> waypoints;
    if (wp_x && wp_z && wp_count > 0) {
        for (int i = 0; i < wp_count; ++i) {
            waypoints.push_back({wp_x[i], wp_z[i], 10.0});
        }
    }

    vehicle_sim::Pose2D ego_pose{ego_x, ego_z, ego_heading, ego_speed};
    vehicle_sim::SteeringOutput steering_output = ctx->pure_pursuit.computeSteering(ego_pose, waypoints);

    vehicle_sim::CoreInput input;
    input.ego_state = {ego_speed, ego_accel, ego_z};
    input.obstacle = {obstacle_pos, obstacle_speed, obstacle_detected};
    input.auto_drive = auto_drive;
    input.manual_input = manual_input;
    input.destination_b = destination_b;
    input.vehicle_length = 4.0;
    input.obstacle_length = 4.0;

    vehicle_sim::CoreOutput core_output = ctx->controller.evaluate(input);
    vehicle_sim::ActuatorAdapter adapter;
    vehicle_sim::ActuatorCommand cmd = adapter.convert(core_output, steering_output);

    if (out_throttle) *out_throttle = cmd.throttle_demand;
    if (out_brake) *out_brake = cmd.brake_demand;
    if (out_steering) *out_steering = cmd.steering_demand;
    if (out_emergency) *out_emergency = cmd.emergency_braking;
    if (out_valid) *out_valid = cmd.valid;
    if (out_fault) *out_fault = cmd.fault_detected;
    if (out_route_completed) *out_route_completed = steering_output.route_completed;
    if (out_cross_track_error) *out_cross_track_error = steering_output.cross_track_error;
}

} // extern "C"
