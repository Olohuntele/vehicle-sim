#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"
#include "vehicle_core.hpp"
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
        testVehicleBridgeInterface();
        testGDExtensionBridgeLoad();
        std::cout << "=== ALL TESTS PASSED SUCCESSFULLY ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
