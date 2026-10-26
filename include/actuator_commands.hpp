#pragma once

#include "vehicle_core.hpp"
#include "pure_pursuit.hpp"
#include <cmath>
#include <string>
#include <algorithm>

namespace vehicle_sim {

struct ActuatorCommand {
    double throttle_demand = 0.0;    // Range [0.0, 1.0] (abstract normalized demand)
    double brake_demand = 0.0;       // Range [0.0, 1.0] (abstract normalized demand)
    double steering_demand = 0.0;    // Range [-1.0, 1.0] (+ means left, - means right, 0.0 = neutral)
    bool emergency_braking = false;  // AEB / emergency priority flag
    bool valid = true;               // Validity flag
    bool fault_detected = false;     // Fault or invalidity indicator
    std::string fault_reason = "";   // Description of fault if invalid/faulted
};

class ActuatorAdapter {
public:
    struct Config {
        double default_brake_on_fault = 1.0;
    };

    ActuatorAdapter(const Config& config = Config()) : config_(config) {}

    /**
     * @brief Converts longitudinal CoreOutput to hardware-independent longitudinal actuator demands
     * with default neutral steering (preserving full backward compatibility with Stage 3C).
     */
    ActuatorCommand convert(const CoreOutput& core_output) {
        SteeringOutput default_steering;
        default_steering.steering_command = 0.0;
        default_steering.valid = true;
        default_steering.fault_detected = false;
        return convert(core_output, default_steering);
    }

    /**
     * @brief Converts vehicle CoreOutput and Pure Pursuit SteeringOutput to hardware-independent
     * longitudinal and lateral actuator demands.
     * Enforces finite values, bounded ranges for throttle/brake [0.0, 1.0] and steering [-1.0, 1.0]
     * (+ means left, - means right), mutual exclusion (throttle vs brake), emergency braking priority,
     * steering fault handling (centering steering as fallback with safe braking on fault), and backward compatibility.
     */
    ActuatorCommand convert(const CoreOutput& core_output, const SteeringOutput& steering_output) {
        ActuatorCommand cmd;

        // 1. Validate longitudinal CoreOutput control_input finiteness
        if (!std::isfinite(core_output.control_input)) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Non-finite control_input detected in CoreOutput";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = config_.default_brake_on_fault;
            cmd.steering_demand = 0.0; // Centered fallback on fault
            cmd.emergency_braking = true;
            return cmd;
        }

        // 2. Validate steering output finiteness, validity, and fault status
        if (!steering_output.valid || steering_output.fault_detected ||
            !std::isfinite(steering_output.steering_command) || !std::isfinite(steering_output.steering_angle)) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = steering_output.fault_detected ?
                               ("Steering fault: " + steering_output.fault_reason) :
                               "Non-finite or invalid steering demand/angle detected";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = config_.default_brake_on_fault;
            cmd.steering_demand = 0.0; // Safe simulated response: steering centered as fallback on fault
            cmd.emergency_braking = true;
            return cmd;
        }

        // 3. Handle emergency braking / AEB priority
        if (core_output.aeb_triggered || core_output.braking_state == BrakingState::EMERGENCY_AEB) {
            cmd.emergency_braking = true;
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = 1.0; // Maximum brake demand
            cmd.steering_demand = std::clamp(steering_output.steering_command, -1.0, 1.0);
            cmd.valid = true;
            cmd.fault_detected = false;
            return cmd;
        }

        // 4. Map control_input [-1.0, 1.0] to throttle and brake demands with mutual exclusion
        double ctrl = core_output.control_input;

        if (ctrl > 0.0) {
            cmd.throttle_demand = std::clamp(ctrl, 0.0, 1.0);
            cmd.brake_demand = 0.0;
        } else if (ctrl < 0.0) {
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = std::clamp(-ctrl, 0.0, 1.0);
        } else {
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = 0.0;
        }

        // 5. Handle specific braking states (planned deceleration, stopping at destination)
        if (core_output.braking_state == BrakingState::STOPPING_AT_DESTINATION ||
            core_output.braking_state == BrakingState::PLANNED_DECEL) {
            if (core_output.control_input >= 0.0) {
                cmd.brake_demand = std::max(cmd.brake_demand, 0.5);
                cmd.throttle_demand = 0.0;
            }
        }

        // 6. Map and clamp steering demand [-1.0, 1.0] (+ means left, - means right)
        cmd.steering_demand = std::clamp(steering_output.steering_command, -1.0, 1.0);

        // 7. Enforce mutual exclusion and strict bounds [0.0, 1.0] for throttle/brake
        cmd.throttle_demand = std::clamp(cmd.throttle_demand, 0.0, 1.0);
        cmd.brake_demand = std::clamp(cmd.brake_demand, 0.0, 1.0);

        if (cmd.throttle_demand > 0.0 && cmd.brake_demand > 0.0) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Mutual exclusion violation: both throttle and brake demands active simultaneously";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = 1.0;
            cmd.steering_demand = 0.0; // Centered fallback on fault
            cmd.emergency_braking = true;
            return cmd;
        }

        // 8. Final finiteness verification on computed outputs
        if (!std::isfinite(cmd.throttle_demand) || !std::isfinite(cmd.brake_demand) || !std::isfinite(cmd.steering_demand)) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Computed actuator demand non-finite";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = config_.default_brake_on_fault;
            cmd.steering_demand = 0.0; // Centered fallback on fault
            cmd.emergency_braking = true;
            return cmd;
        }

        cmd.valid = true;
        cmd.fault_detected = false;
        return cmd;
    }

private:
    Config config_;
};

} // namespace vehicle_sim
