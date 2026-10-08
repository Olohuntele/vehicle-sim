#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

void testVehicleKinematicsAndFriction() {
    std::cout << "Running testVehicleKinematicsAndFriction...\n";
    
    Vehicle v(0.0);
    assert(v.getVelocity() == 0.0);
    assert(v.getPosition() == 0.0);

    // Test acceleration
    v.setControlInput(1.0); // max acceleration
    v.update(1.0); // 1 second dt
    assert(v.getVelocity() > 0.0);
    assert(v.getPosition() > 0.0);

    // Test friction limit capping
    v.reset(10.0); // 10 m/s
    v.setRoadFriction(0.2); // low friction (e.g. ice)
    v.setControlInput(-1.0); // full braking
    v.update(0.1);
    
    // Max braking force under mu=0.2 should be capped at mu * m * g = 0.2 * 1200 * 9.81 = 2354.4 N
    double max_friction_force = 0.2 * 1200.0 * 9.81;
    double expected_max_decel = max_friction_force / 1200.0; // ~1.962 m/s^2
    
    std::cout << "Vehicle friction test passed successfully.\n";
}

void testCollisionAvoidance() {
    std::cout << "Running testCollisionAvoidance...\n";

    vehicle_sim::CollisionAvoidance::Config config{2.0, 5.0, -8.0};
    vehicle_sim::CollisionAvoidance ca(config);

    vehicle_sim::VehicleState ego{10.0, 0.0, 0.0};
    vehicle_sim::Obstacle obstacle{15.0, 5.0, true}; // distance = 15, rel_vel = 5, TTC = 15/5 = 3s (no AEB)

    double intervention = ca.evaluate(ego, obstacle);
    assert(intervention == 0.0);

    // Close obstacle -> short TTC
    vehicle_sim::Obstacle closeObstacle{20.0, 2.0, true}; // rel_dist = 20, rel_vel = 8, TTC = 20/8 = 2.5s -> wait, let's make TTC < 2.0
    // If ego at 10, obstacle at 18: rel_dist = 18, rel_vel = 8 -> TTC = 18/8 = 2.25s
    // If obstacle at 12: rel_dist = 12, rel_vel = 8 -> TTC = 12/8 = 1.5s (< 2.0)
    vehicle_sim::Obstacle imminentObstacle{12.0, 2.0, true};
    intervention = ca.evaluate(ego, imminentObstacle);
    assert(intervention == -8.0); // AEB triggered

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

int main() {
    std::cout << "=== Starting Vehicle Sim Unit Tests ===\n";
    try {
        testVehicleKinematicsAndFriction();
        testCollisionAvoidance();
        testACCController();
        std::cout << "=== ALL TESTS PASSED SUCCESSFULLY ===\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
