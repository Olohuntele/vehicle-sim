#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"
#include "vehicle_core.hpp"
#include "actuator_commands.hpp"
#include "pure_pursuit.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

// Declare C bridge functions for testing
extern "C" {
    void* create_controller(double ttc_threshold, double min_safe_dist, double max_braking, double target_speed, double time_gap);
    void destroy_controller(void* handle);
    void evaluate_step(void* handle,
                       double ego_pos, double ego_speed, double ego_accel,
                       bool obstacle_detected, double obstacle_pos, double obstacle_speed,
                       bool auto_drive, double manual_input, double destination_b,
                       double* out_control_input, bool* out_aeb, bool* out_acc_active,
                       double* out_bumper_clearance, double* out_stopping_dist, int* out_warning_level);
    void evaluate_actuator_step(void* handle,
                                double ego_pos, double ego_speed, double ego_accel,
                                bool obstacle_detected, double obstacle_pos, double obstacle_speed,
                                bool auto_drive, double manual_input, double destination_b,
                                double* out_throttle, double* out_brake, bool* out_emergency,
                                bool* out_valid, bool* out_fault);
}

void testVehicleKinematicsAndFriction() {
    std::cout << "Running testVehicleKinematicsAndFriction...\n";
    Vehicle v(0.0);
    assert(v.getVelocity() == 0.0);
    assert(v.getPosition() == 0.0);
    v.setControlInput(1.0);
    v.update(1.0);
    assert(v.getVelocity() > 0.0);
    assert(v.getPosition() > 0.0);
    std::cout << "Vehicle friction test passed successfully.\n";
}

void testCollisionAvoidance() {
    std::cout << "Running testCollisionAvoidance...\n";
    vehicle_sim::CollisionAvoidance::Config config{2.0, 5.0, -8.0};
    vehicle_sim::CollisionAvoidance ca(config);
    vehicle_sim::VehicleState ego{10.0, 0.0, 0.0};
    vehicle_sim::Obstacle obstacle{15.0, 5.0, true};
    double intervention = ca.evaluate(ego, obstacle);
    assert(intervention == 0.0);
    std::cout << "Collision avoidance test passed successfully.\n";
}

void testACCController() {
    std::cout << "Running testACCController...\n";
    vehicle_sim::ACCController::Config config{10.0, 1.5, 0.5, 0.3};
    vehicle_sim::ACCController acc(config);
    vehicle_sim::VehicleState ego{5.0, 0.0, 0.0};
    vehicle_sim::Obstacle obstacle{50.0, 8.0, true};
    double control = acc.computeControl(ego, obstacle);
    assert(control >= -1.0 && control <= 1.0);
    std::cout << "ACC controller test passed successfully.\n";
}

void testActuatorAdapter() {
    std::cout << "Running testActuatorAdapter (Stage 3B Actuator Command Interface)...\n";
    vehicle_sim::ActuatorAdapter adapter;

    // 1. Acceleration test
    {
        vehicle_sim::CoreOutput out;
        out.control_input = 0.75;
        out.aeb_triggered = false;
        out.braking_state = vehicle_sim::BrakingState::ACCELERATING;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == true);
        assert(cmd.fault_detected == false);
        assert(cmd.emergency_braking == false);
        assert(cmd.throttle_demand == 0.75);
        assert(cmd.brake_demand == 0.0);
    }

    // 2. Coasting test
    {
        vehicle_sim::CoreOutput out;
        out.control_input = 0.0;
        out.aeb_triggered = false;
        out.braking_state = vehicle_sim::BrakingState::COASTING;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == true);
        assert(cmd.throttle_demand == 0.0);
        assert(cmd.brake_demand == 0.0);
    }

    // 3. Braking / Planned Decel test
    {
        vehicle_sim::CoreOutput out;
        out.control_input = -0.4;
        out.aeb_triggered = false;
        out.braking_state = vehicle_sim::BrakingState::PLANNED_DECEL;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == true);
        assert(cmd.throttle_demand == 0.0);
        assert(cmd.brake_demand >= 0.4);
        assert(!(cmd.throttle_demand > 0.0 && cmd.brake_demand > 0.0)); // Mutual exclusion
    }

    // 4. AEB Priority test
    {
        vehicle_sim::CoreOutput out;
        out.control_input = 0.5; // conflicting throttle request
        out.aeb_triggered = true;
        out.braking_state = vehicle_sim::BrakingState::EMERGENCY_AEB;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == true);
        assert(cmd.emergency_braking == true);
        assert(cmd.brake_demand == 1.0);
        assert(cmd.throttle_demand == 0.0);
    }

    // 5. Invalid Input (Non-finite NaN) test
    {
        vehicle_sim::CoreOutput out;
        out.control_input = NAN;
        out.aeb_triggered = false;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == false);
        assert(cmd.fault_detected == true);
        assert(cmd.emergency_braking == true);
        assert(cmd.brake_demand == 1.0); // Fail-safe default brake
        assert(cmd.throttle_demand == 0.0);
    }

    // 6. Bounds check & Mutual exclusion
    {
        vehicle_sim::CoreOutput out;
        out.control_input = 2.5; // out of normal range [0, 1]
        out.aeb_triggered = false;
        out.braking_state = vehicle_sim::BrakingState::CRUISING;

        auto cmd = adapter.convert(out);
        assert(cmd.valid == true);
        assert(cmd.throttle_demand <= 1.0);
        assert(cmd.throttle_demand >= 0.0);
        assert(cmd.brake_demand == 0.0);
    }

    std::cout << "Actuator adapter tests passed successfully.\n";
}

void testVehicleBridgeInterface() {
    std::cout << "Running testVehicleBridgeInterface (Phase 2A C-Bridge Validation)...\n";
    void* handle = create_controller(2.0, 5.0, 8.0, 10.0, 1.5);
    assert(handle != nullptr);

    double control_input = 0.0;
    bool aeb = false;
    bool acc_active = false;
    double clearance = 0.0;
    double stop_dist = 0.0;
    int warning_level = 0;

    // Test evaluation via C bridge
    evaluate_step(handle, 0.0, 5.0, 0.0, true, 60.0, 0.0, true, 0.0, 100.0,
                  &control_input, &aeb, &acc_active, &clearance, &stop_dist, &warning_level);

    assert(acc_active == true);
    assert(aeb == false);
    assert(clearance > 0.0);

    // Test imminent obstacle via C bridge
    evaluate_step(handle, 10.0, 10.0, 0.0, true, 12.0, 2.0, true, 0.0, 100.0,
                  &control_input, &aeb, &acc_active, &clearance, &stop_dist, &warning_level);

    assert(aeb == true);
    assert(control_input == -1.0);

    destroy_controller(handle);
    std::cout << "Vehicle bridge interface tests passed successfully.\n";
}

void testVehicleBridgeActuatorStep() {
    std::cout << "Running testVehicleBridgeActuatorStep (Stage 3C Bridge Actuator Step)...\n";
    void* handle = create_controller(2.0, 5.0, 8.0, 10.0, 1.5);
    assert(handle != nullptr);

    double throttle = 0.0;
    double brake = 0.0;
    bool emergency = false;
    bool valid = false;
    bool fault = true;

    // Test actuator evaluation via C bridge under AEB condition
    evaluate_actuator_step(handle, 10.0, 10.0, 0.0, true, 12.0, 2.0, true, 0.0, 100.0,
                           &throttle, &brake, &emergency, &valid, &fault);

    assert(valid == true);
    assert(fault == false);
    assert(emergency == true);
    assert(brake == 1.0);
    assert(throttle == 0.0);

    destroy_controller(handle);
    std::cout << "Vehicle bridge actuator step tests passed successfully.\n";
}

void testStage3CSimulationScenarios() {
    std::cout << "Running testStage3CSimulationScenarios (Stage 3C Automated Simulation Verification)...\n";

    double max_accel = 3.5;
    double max_brake = 6.0;
    double dt = 0.05; // 50ms simulation step

    // --- Scenario 1: Blocked Road (Obstacle at Z=60, Initial Speed = 0) ---
    {
        double ego_pos = 0.0;
        double ego_speed = 0.0;
        double lead_pos = 60.0;
        double ego_length = 4.0;
        double lead_length = 4.0;
        bool collision = false;
        double min_clearance = 999.0;
        int max_steps = 600; // 30 seconds max

        vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -6.0};
        vehicle_sim::ACCController::Config accConfig{10.0, 1.5, 0.5, 0.3};
        vehicle_sim::VehicleCoreController controller(caConfig, accConfig);
        vehicle_sim::ActuatorAdapter adapter;

        int steps_run = 0;
        for (int i = 0; i < max_steps; ++i) {
            steps_run++;
            vehicle_sim::CoreInput input;
            input.ego_state = {ego_speed, 0.0, ego_pos};
            input.obstacle = {lead_pos, 0.0, true};
            input.auto_drive = true;
            input.manual_input = 0.0;
            input.destination_b = 100.0;
            input.vehicle_length = ego_length;
            input.obstacle_length = lead_length;

            vehicle_sim::CoreOutput out = controller.evaluate(input);
            vehicle_sim::ActuatorCommand cmd = adapter.convert(out);

            double lead_rear = lead_pos - lead_length / 2.0;
            double ego_front = ego_pos + ego_length / 2.0;
            double clearance = lead_rear - ego_front;
            if (clearance < min_clearance) min_clearance = clearance;
            if (clearance <= 0.0) collision = true;

            double accel = 0.0;
            if (!cmd.valid || cmd.fault_detected) {
                accel = -max_brake;
            } else if (cmd.emergency_braking || cmd.brake_demand > 0.0) {
                accel = -std::max(cmd.brake_demand, 0.5) * max_brake;
            } else if (cmd.throttle_demand > 0.0) {
                accel = cmd.throttle_demand * max_accel;
            } else {
                accel = -0.1;
            }

            ego_speed = std::max(0.0, ego_speed + accel * dt);
            ego_pos += ego_speed * dt;

            if (ego_speed < 0.01 && ego_pos > 10.0 && out.braking_state == vehicle_sim::BrakingState::EMERGENCY_AEB) {
                break;
            }
            if (ego_speed < 0.001 && ego_pos > 45.0) {
                break;
            }
        }

        std::cout << "  [Scenario 1: Blocked Road @ Z=60]\n";
        std::cout << "    Stopping Position Z: " << ego_pos << " m\n";
        std::cout << "    Final Speed: " << ego_speed << " m/s\n";
        std::cout << "    Min Bumper Clearance: " << min_clearance << " m\n";
        std::cout << "    Collision Result: " << (collision ? "COLLISION (FAIL)" : "SAFE (PASS)") << "\n";

        assert(!collision);
        assert(ego_speed < 0.1);
        assert(min_clearance > 0.0);
    }

    // --- Scenario 2: Clear Road to Destination B (Z=100) ---
    {
        double ego_pos = 0.0;
        double ego_speed = 0.0;
        double destination_b = 100.0;
        double ego_length = 4.0;
        double min_clearance = 999.0;
        int max_steps = 800;

        vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -6.0};
        vehicle_sim::ACCController::Config accConfig{10.0, 1.5, 0.5, 0.3};
        vehicle_sim::VehicleCoreController controller(caConfig, accConfig);
        vehicle_sim::ActuatorAdapter adapter;

        for (int i = 0; i < max_steps; ++i) {
            vehicle_sim::CoreInput input;
            input.ego_state = {ego_speed, 0.0, ego_pos};
            input.obstacle = {999.0, 0.0, false};
            input.auto_drive = true;
            input.manual_input = 0.0;
            input.destination_b = destination_b;
            input.vehicle_length = ego_length;

            vehicle_sim::CoreOutput out = controller.evaluate(input);
            vehicle_sim::ActuatorCommand cmd = adapter.convert(out);

            double ego_front = ego_pos + ego_length / 2.0;
            double clearance = destination_b - ego_front;
            if (clearance < min_clearance) min_clearance = clearance;

            double accel = 0.0;
            if (!cmd.valid || cmd.fault_detected) {
                accel = -max_brake;
            } else if (cmd.emergency_braking || cmd.brake_demand > 0.0) {
                accel = -std::max(cmd.brake_demand, 0.5) * max_brake;
            } else if (cmd.throttle_demand > 0.0) {
                accel = cmd.throttle_demand * max_accel;
            } else {
                accel = -0.1;
            }

            ego_speed = std::max(0.0, ego_speed + accel * dt);
            ego_pos += ego_speed * dt;

            if (ego_speed < 0.01 && ego_pos >= 90.0) {
                break;
            }
        }

        std::cout << "  [Scenario 2: Clear Road to Destination B @ Z=100]\n";
        std::cout << "    Stopping Position Z: " << ego_pos << " m\n";
        std::cout << "    Final Speed: " << ego_speed << " m/s\n";
        std::cout << "    Min Bumper Clearance: " << min_clearance << " m\n";

        assert(ego_speed < 0.1);
        assert(ego_pos <= destination_b + 2.0);
    }

    // --- Scenario 3: High Initial Speed (v=15 m/s approaching obstacle at Z=60) ---
    {
        double ego_pos = 0.0;
        double ego_speed = 15.0;
        double lead_pos = 60.0;
        double ego_length = 4.0;
        double lead_length = 4.0;
        bool collision = false;
        double min_clearance = 999.0;
        int max_steps = 500;

        vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -6.0};
        vehicle_sim::ACCController::Config accConfig{10.0, 1.5, 0.5, 0.3};
        vehicle_sim::VehicleCoreController controller(caConfig, accConfig);
        vehicle_sim::ActuatorAdapter adapter;

        for (int i = 0; i < max_steps; ++i) {
            vehicle_sim::CoreInput input;
            input.ego_state = {ego_speed, 0.0, ego_pos};
            input.obstacle = {lead_pos, 0.0, true};
            input.auto_drive = true;
            input.manual_input = 0.0;
            input.destination_b = 100.0;
            input.vehicle_length = ego_length;
            input.obstacle_length = lead_length;

            vehicle_sim::CoreOutput out = controller.evaluate(input);
            vehicle_sim::ActuatorCommand cmd = adapter.convert(out);

            double lead_rear = lead_pos - lead_length / 2.0;
            double ego_front = ego_pos + ego_length / 2.0;
            double clearance = lead_rear - ego_front;
            if (clearance < min_clearance) min_clearance = clearance;
            if (clearance <= 0.0) collision = true;

            double accel = 0.0;
            if (!cmd.valid || cmd.fault_detected) {
                accel = -max_brake;
            } else if (cmd.emergency_braking || cmd.brake_demand > 0.0) {
                accel = -std::max(cmd.brake_demand, 0.5) * max_brake;
            } else if (cmd.throttle_demand > 0.0) {
                accel = cmd.throttle_demand * max_accel;
            } else {
                accel = -0.1;
            }

            ego_speed = std::max(0.0, ego_speed + accel * dt);
            ego_pos += ego_speed * dt;

            if (ego_speed < 0.01 && ego_pos > 10.0) {
                break;
            }
        }

        std::cout << "  [Scenario 3: High Initial Speed v=15 m/s @ Obstacle Z=60]\n";
        std::cout << "    Stopping Position Z: " << ego_pos << " m\n";
        std::cout << "    Final Speed: " << ego_speed << " m/s\n";
        std::cout << "    Min Bumper Clearance: " << min_clearance << " m\n";
        std::cout << "    Collision Result: " << (collision ? "COLLISION (FAIL)" : "SAFE (PASS)") << "\n";

        assert(!collision);
        assert(ego_speed < 0.1);
        assert(min_clearance > 0.0);
    }

    // --- Scenario 4: Invalid Actuator Output / Fail-Safe Response ---
    {
        vehicle_sim::ActuatorAdapter adapter;
        vehicle_sim::CoreOutput corrupted_out;
        corrupted_out.control_input = NAN;

        vehicle_sim::ActuatorCommand cmd = adapter.convert(corrupted_out);

        std::cout << "  [Scenario 4: Invalid Actuator Output / Fail-Safe Response]\n";
        std::cout << "    Valid Flag: " << (cmd.valid ? "true" : "false") << "\n";
        std::cout << "    Fault Detected: " << (cmd.fault_detected ? "true" : "false") << "\n";
        std::cout << "    Emergency Braking: " << (cmd.emergency_braking ? "true" : "false") << "\n";
        std::cout << "    Throttle Demand: " << cmd.throttle_demand << "\n";
        std::cout << "    Brake Demand: " << cmd.brake_demand << "\n";
        std::cout << "    Fault Reason: " << cmd.fault_reason << "\n";

        assert(cmd.valid == false);
        assert(cmd.fault_detected == true);
        assert(cmd.emergency_braking == true);
        assert(cmd.throttle_demand == 0.0);
        assert(cmd.brake_demand == 1.0);
    }

    std::cout << "All Stage 3C simulation scenarios verified successfully (PASS).\n";
}

void testPurePursuitController() {
    std::cout << "Running testPurePursuitController (Stage 4A Pure Pursuit Steering Controller)...\n";

    // 1. Straight Tracking Test
    {
        vehicle_sim::PurePursuitController controller;
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 10.0}; // x=0, z=0, heading=0, v=10
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {0.0, 10.0, 10.0},
            {0.0, 20.0, 10.0},
            {0.0, 30.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(!output.fault_detected);
        assert(std::abs(output.steering_command) < 0.05); // Should steer straight (approx 0)
        assert(std::abs(output.steering_angle) < 0.05);
    }

    // 2. Left Turn Test (Waypoints curving to the left / negative X)
    {
        vehicle_sim::PurePursuitController controller;
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 5.0};
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {-5.0, 10.0, 10.0},
            {-10.0, 20.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(!output.fault_detected);
        assert(output.steering_angle > 0.0);    // Left turn requires positive steering angle (turn left)
        assert(output.steering_command > 0.0);  // Left turn requires positive normalized command
    }

    // 3. Right Turn Test (Waypoints curving to the right / positive X)
    {
        vehicle_sim::PurePursuitController controller;
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 5.0};
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {5.0, 10.0, 10.0},
            {10.0, 20.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(!output.fault_detected);
        assert(output.steering_angle < 0.0);    // Right turn requires negative steering angle (turn right)
        assert(output.steering_command < 0.0);  // Right turn requires negative normalized command
    }

    // 4. Duplicate / Degenerate Waypoint Test
    {
        vehicle_sim::PurePursuitController controller;
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 5.0};
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {0.0, 0.0, 10.0}, // Duplicate waypoint
            {0.0, 10.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(!output.fault_detected);
    }

    // 4. Steering Bounds Test
    {
        vehicle_sim::PurePursuitConfig config;
        config.max_steering_angle = 0.5;
        vehicle_sim::PurePursuitController controller(config);
        // Extreme lateral offset
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 5.0};
        std::vector<vehicle_sim::Waypoint> route = {
            {50.0, 5.0, 10.0},
            {50.0, 20.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(output.steering_angle >= -0.5 && output.steering_angle <= 0.5);
        assert(output.steering_command >= -1.0 && output.steering_command <= 1.0);
    }

    // 5. Waypoint Progression and Route Completion Test
    {
        vehicle_sim::PurePursuitController controller;
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {0.0, 5.0, 10.0},
            {0.0, 10.0, 10.0}
        };

        // Pose near final waypoint
        vehicle_sim::Pose2D pose{0.0, 9.5, 0.0, 0.0};
        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(output.route_completed == true);
        assert(output.steering_command == 0.0);
    }

    // 6. Zero-Speed Operation Test
    {
        vehicle_sim::PurePursuitController controller;
        vehicle_sim::Pose2D pose{0.0, 0.0, 0.0, 0.0}; // v = 0.0
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 10.0},
            {0.0, 10.0, 10.0}
        };

        auto output = controller.computeSteering(pose, route);
        assert(output.valid == true);
        assert(!output.fault_detected);
    }

    // 7. Invalid Inputs Test (NaN, Inf, Empty Route)
    {
        vehicle_sim::PurePursuitController controller;
        // NaN pose
        vehicle_sim::Pose2D bad_pose{NAN, 0.0, 0.0, 10.0};
        std::vector<vehicle_sim::Waypoint> route = {{0.0, 0.0}, {0.0, 10.0}};

        auto output1 = controller.computeSteering(bad_pose, route);
        assert(output1.valid == false);
        assert(output1.fault_detected == true);

        // Empty route
        std::vector<vehicle_sim::Waypoint> empty_route;
        vehicle_sim::Pose2D good_pose{0.0, 0.0, 0.0, 10.0};
        auto output2 = controller.computeSteering(good_pose, empty_route);
        assert(output2.valid == true);
        assert(output2.route_completed == true);
    }

    // 8. Kinematic Bicycle Model Test Harness Simulation
    {
        vehicle_sim::PurePursuitConfig config;
        config.wheelbase = 2.5;
        config.lookahead_distance = 4.0;
        config.max_steering_angle = 0.6;
        vehicle_sim::PurePursuitController controller(config);

        vehicle_sim::Pose2D ego{0.0, 0.0, 0.0, 5.0}; // v = 5 m/s
        std::vector<vehicle_sim::Waypoint> route = {
            {0.0, 0.0, 5.0},
            {0.0, 15.0, 5.0},
            {0.0, 30.0, 5.0}
        };

        double dt = 0.1;
        bool reached = false;
        for (int step = 0; step < 200; ++step) {
            auto steering_out = controller.computeSteering(ego, route);
            assert(steering_out.valid);

            if (steering_out.route_completed) {
                reached = true;
                break;
            }

            // Kinematic bicycle model update
            double delta = steering_out.steering_angle;
            double v = ego.velocity;
            double L = config.wheelbase;

            double d_heading = (v / L) * std::tan(delta) * dt;
            ego.heading += d_heading;
            ego.x += v * std::sin(ego.heading) * dt;
            ego.z += v * std::cos(ego.heading) * dt;
        }

        std::cout << "  [Bicycle Model Simulation Test]: Route completed = " << (reached ? "true" : "false") << "\n";
        assert(reached);
    }

    std::cout << "Pure Pursuit controller tests passed successfully.\n";
}

void testStage4BSteeringActuatorValidation() {
    std::cout << "Running testStage4BSteeringActuatorValidation (Stage 4B Steering Actuator Validation & Fault Handling)...\n";
    vehicle_sim::ActuatorAdapter adapter;

    // 1. Valid Left Steering Test (+ demand)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;
        core_out.braking_state = vehicle_sim::BrakingState::ACCELERATING;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = 0.6; // Left turn
        steering_out.steering_angle = 0.3;
        steering_out.valid = true;
        steering_out.fault_detected = false;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == true);
        assert(cmd.fault_detected == false);
        assert(cmd.steering_demand == 0.6);
        assert(cmd.throttle_demand == 0.5);
    }

    // 2. Valid Right Steering Test (- demand)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;
        core_out.braking_state = vehicle_sim::BrakingState::ACCELERATING;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = -0.4; // Right turn
        steering_out.steering_angle = -0.2;
        steering_out.valid = true;
        steering_out.fault_detected = false;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == true);
        assert(cmd.steering_demand == -0.4);
    }

    // 3. Straight Steering Test (0 demand)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.0;
        core_out.braking_state = vehicle_sim::BrakingState::COASTING;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = 0.0;
        steering_out.steering_angle = 0.0;
        steering_out.valid = true;
        steering_out.fault_detected = false;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == true);
        assert(cmd.steering_demand == 0.0);
    }

    // 4. Steering Saturation Test (clamping out-of-bounds demand > 1.0 or < -1.0)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = 2.5; // Out of bounds
        steering_out.steering_angle = 1.2;
        steering_out.valid = true;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == true);
        assert(cmd.steering_demand == 1.0); // Clamped to 1.0
    }

    // 5. Invalid Values (NaN / Inf) Steering Test
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = NAN;
        steering_out.steering_angle = 0.2;
        steering_out.valid = true;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == false);
        assert(cmd.fault_detected == true);
        assert(cmd.steering_demand == 0.0); // Centered fallback on fault
        assert(cmd.brake_demand == 1.0);
    }

    // 6. Fault Propagation & Safe Simulated Response Test
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.valid = false;
        steering_out.fault_detected = true;
        steering_out.fault_reason = "Pure Pursuit sensor fault";

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == false);
        assert(cmd.fault_detected == true);
        assert(cmd.steering_demand == 0.0); // Steering centered as fallback
        assert(cmd.throttle_demand == 0.0); // No propulsion
        assert(cmd.brake_demand == 1.0);    // Safe braking applied
    }

    // 7. AEB Priority Preservation Test (AEB active retains valid steering while forcing max brake)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.5;
        core_out.aeb_triggered = true;
        core_out.braking_state = vehicle_sim::BrakingState::EMERGENCY_AEB;

        vehicle_sim::SteeringOutput steering_out;
        steering_out.steering_command = 0.5;
        steering_out.valid = true;

        auto cmd = adapter.convert(core_out, steering_out);
        assert(cmd.valid == true);
        assert(cmd.emergency_braking == true);
        assert(cmd.brake_demand == 1.0);
        assert(cmd.throttle_demand == 0.0);
        assert(cmd.steering_demand == 0.5); // Steering demand preserved during AEB
    }

    // 8. Stage 3C Backward Compatibility Test (longitudinal-only caller)
    {
        vehicle_sim::CoreOutput core_out;
        core_out.control_input = 0.8;
        core_out.braking_state = vehicle_sim::BrakingState::ACCELERATING;

        auto cmd = adapter.convert(core_out); // Single argument overload (Stage 3C style)
        assert(cmd.valid == true);
        assert(cmd.throttle_demand == 0.8);
        assert(cmd.brake_demand == 0.0);
        assert(cmd.steering_demand == 0.0); // Default neutral steering
    }

    std::cout << "Stage 4B steering actuator validation tests passed successfully.\n";
}

#ifdef _WIN32
#include <windows.h>
#endif

void testGDExtensionBridgeLoad() {
    std::cout << "Running testGDExtensionBridgeLoad (Phase 2A.4 GDExtension DLL Validation)...\n";
#ifdef _WIN32
    HMODULE hMod = LoadLibraryA("godot_sim/bin/easydrive_extension.dll");
    assert(hMod != nullptr && "Failed to load easydrive_extension.dll");
    
    FARPROC initProc = GetProcAddress(hMod, "easydrive_library_init");
    assert(initProc != nullptr && "Failed to find easydrive_library_init entry point in DLL");
    
    std::cout << "Successfully loaded easydrive_extension.dll and resolved entry point 'easydrive_library_init'.\n";
    FreeLibrary(hMod);
#else
    std::cout << "Skipping Windows GDExtension load test on non-Windows platform.\n";
#endif
    std::cout << "GDExtension bridge load test passed successfully.\n";
}

int main() {
    std::cout << "=== Starting Vehicle Core, Bridge & Trajectory Tests ===\n";
    try {
        testVehicleKinematicsAndFriction();
        testCollisionAvoidance();
        testACCController();
        testActuatorAdapter();
        testVehicleBridgeInterface();
        testVehicleBridgeActuatorStep();
        testStage3CSimulationScenarios();
        testPurePursuitController();
        testStage4BSteeringActuatorValidation();
        testGDExtensionBridgeLoad();
        std::cout << "=== ALL TESTS PASSED SUCCESSFULLY ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
