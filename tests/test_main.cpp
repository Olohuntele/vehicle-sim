#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"
#include "vehicle_core.hpp"
#include "actuator_commands.hpp"
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
        testGDExtensionBridgeLoad();
        std::cout << "=== ALL TESTS PASSED SUCCESSFULLY ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
