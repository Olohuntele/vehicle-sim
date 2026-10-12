#pragma once

#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"

namespace vehicle_sim {

enum class WarningLevel {
    LEVEL_BLUE,   // Normal cruising / accelerating
    LEVEL_YELLOW, // Caution / approaching obstacle or planned deceleration
    LEVEL_RED     // AEB active / emergency braking / critical proximity
};

enum class BrakingState {
    ACCELERATING,
    CRUISING,
    PLANNED_DECEL,
    EMERGENCY_AEB,
    STOPPING_AT_DESTINATION,
    COASTING
};

struct CoreInput {
    VehicleState ego_state;
    Obstacle obstacle;
    bool auto_drive = false;
    double manual_input = 0.0;
    bool key_left_pressed = false;
    bool key_right_pressed = false;
    double vehicle_length = 4.0;
    double obstacle_length = 4.0;
    double destination_b = 100.0;
    double system_delay = 0.1;     // s
    double safety_margin = 2.0;    // m
    double max_braking = 6.0;      // m/s^2
};

struct CoreOutput {
    double control_input = 0.0;
    bool aeb_triggered = false;
    bool acc_active = false;
    double relative_distance = 0.0;
    double relative_speed = 0.0;
    double ttc = 99.9;
    double bumper_clearance = 0.0;
    double estimated_stopping_distance = 0.0;
    WarningLevel warning_level = WarningLevel::LEVEL_BLUE;
    BrakingState braking_state = BrakingState::COASTING;
    bool collision_detected = false;
};

class VehicleCoreController {
public:
    VehicleCoreController(const CollisionAvoidance::Config& caConfig,
                          const ACCController::Config& accConfig);

    CoreOutput evaluate(const CoreInput& input);

private:
    CollisionAvoidance collision_avoidance_;
    ACCController acc_controller_;
};

} // namespace vehicle_sim
