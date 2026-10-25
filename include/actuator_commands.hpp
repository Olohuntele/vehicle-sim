#pragma once

#include "vehicle_core.hpp"
#include <cmath>
#include <string>
#include <algorithm>

namespace vehicle_sim {

struct ActuatorCommand {
    double throttle_demand = 0.0;    // Range [0.0, 1.0] (abstract normalized demand)
    double brake_demand = 0.0;       // Range [0.0, 1.0] (abstract normalized demand)
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
     * @brief Converts vehicle CoreOutput to hardware-independent longitudinal actuator demands.
     * Enforces finite values, bounded ranges [0.0, 1.0], mutual exclusion (throttle vs brake),
     * emergency braking priority, and fault handling for invalid inputs.
     * Note: Lateral steering is reserved for future modules and not included.
     */
    ActuatorCommand convert(const CoreOutput& core_output) {
        ActuatorCommand cmd;

        // 1. Validate inputs (check finiteness of control_input)
        if (!std::isfinite(core_output.control_input)) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Non-finite control_input detected in CoreOutput";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = config_.default_brake_on_fault;
            cmd.emergency_braking = true;
            return cmd;
        }

        // 2. Handle emergency braking / AEB priority
        if (core_output.aeb_triggered || core_output.braking_state == BrakingState::EMERGENCY_AEB) {
            cmd.emergency_braking = true;
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = 1.0; // Maximum brake demand
            cmd.valid = true;
            cmd.fault_detected = false;
            return cmd;
        }

        // 3. Map control_input [-1.0, 1.0] to throttle and brake demands with mutual exclusion
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

        // 4. Handle specific braking states (planned deceleration, stopping at destination)
        if (core_output.braking_state == BrakingState::STOPPING_AT_DESTINATION ||
            core_output.braking_state == BrakingState::PLANNED_DECEL) {
            if (core_output.control_input >= 0.0) {
                cmd.brake_demand = std::max(cmd.brake_demand, 0.5);
                cmd.throttle_demand = 0.0;
            }
        }

        // 5. Enforce mutual exclusion and strict bounds [0.0, 1.0]
        cmd.throttle_demand = std::clamp(cmd.throttle_demand, 0.0, 1.0);
        cmd.brake_demand = std::clamp(cmd.brake_demand, 0.0, 1.0);

        if (cmd.throttle_demand > 0.0 && cmd.brake_demand > 0.0) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Mutual exclusion violation: both throttle and brake demands active simultaneously";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = 1.0;
            cmd.emergency_braking = true;
            return cmd;
        }

        // 6. Final finiteness verification on computed outputs
        if (!std::isfinite(cmd.throttle_demand) || !std::isfinite(cmd.brake_demand)) {
            cmd.valid = false;
            cmd.fault_detected = true;
            cmd.fault_reason = "Computed actuator demand non-finite";
            cmd.throttle_demand = 0.0;
            cmd.brake_demand = config_.default_brake_on_fault;
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
