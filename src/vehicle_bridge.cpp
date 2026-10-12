#include "vehicle_core.hpp"

extern "C" {

struct ControllerContext {
    vehicle_sim::VehicleCoreController controller;
    ControllerContext(const vehicle_sim::CollisionAvoidance::Config& ca,
                      const vehicle_sim::ACCController::Config& acc)
        : controller(ca, acc) {}
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

} // extern "C"
