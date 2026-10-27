#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include "actuator_commands.hpp"
#include "pure_pursuit.hpp"

namespace vehicle_sim {
class VehicleCoreController;
class PurePursuitController;
}

namespace godot {

class EasyDriveBridge : public Node {
    GDCLASS(EasyDriveBridge, Node);

protected:
    static void _bind_methods();

public:
    EasyDriveBridge();
    ~EasyDriveBridge();

    String get_bridge_status() const;
    Dictionary evaluate_shadow_step(
        double ego_pos, double ego_speed, double ego_accel,
        bool obstacle_detected, double obstacle_pos, double obstacle_speed,
        bool auto_drive, double manual_input, double destination_b
    );
    Dictionary evaluate_actuator_step(
        double ego_pos, double ego_speed, double ego_accel,
        bool obstacle_detected, double obstacle_pos, double obstacle_speed,
        bool auto_drive, double manual_input, double destination_b
    );
    Dictionary evaluate_path_step(
        double ego_x, double ego_z, double ego_heading, double ego_speed, double ego_accel,
        const Array& waypoints_array,
        bool obstacle_detected, double obstacle_pos, double obstacle_speed,
        bool auto_drive, double manual_input, double destination_b
    );

private:
    vehicle_sim::VehicleCoreController* controller_;
    vehicle_sim::PurePursuitController* pure_pursuit_controller_;
};

} // namespace godot
