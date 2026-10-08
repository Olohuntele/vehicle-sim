#include "raylib.h"
#include "Vehicle.hpp"
#include "collision_avoidance.hpp"
#include "acc_controller.hpp"
#include "behavior_tree.hpp"
#include <iostream>
#include <cmath>

int main() {
    const int screenWidth = 800;
    const int screenHeight = 450;
    
    InitWindow(screenWidth, screenHeight, "Autonomous Vehicle Sim - Behavior Tree & ACC");
    SetTargetFPS(60);

    // Instantiate Ego Vehicle, Safety Module, and ACC Controller
    Vehicle egoVehicle(0.0);
    vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, 8.0};
    vehicle_sim::CollisionAvoidance collisionAvoidance(caConfig);

    vehicle_sim::ACCController::Config accConfig{8.0, 1.5, 0.5, 0.3};
    vehicle_sim::ACCController accController(accConfig);

    // Moving Target Vehicle (Red Obstacle)
    float targetX = 500.0f;
    float targetSpeed = 2.0f;
    const float sensorRange = 350.0f;
    
    bool autoDrive = false;
    const double dt = 1.0 / 60.0;

    // Simulation state variables shared with behavior tree evaluation
    double controlInput = 0.0;
    bool aebTriggered = false;
    bool accActive = false;

    while (!WindowShouldClose()) {
        // --- Controls ---
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyPressed(KEY_SPACE)) {
            autoDrive = !autoDrive;
        }

        // Reset Scenario
        if (IsKeyPressed(KEY_R)) {
            egoVehicle.reset(0.0);
            targetX = 500.0f;
            targetSpeed = 2.0f;
            autoDrive = false;
        }

        // Control Target Vehicle Speed
        if (IsKeyDown(KEY_UP)) targetSpeed += 0.05f;
        if (IsKeyDown(KEY_DOWN)) targetSpeed -= 0.05f;
        if (targetSpeed < 0.0f) targetSpeed = 0.0f;
        if (targetSpeed > 8.0f) targetSpeed = 8.0f;

        // Move Target Vehicle
        targetX += targetSpeed;
        if (targetX > 750.0f) targetX = 750.0f;

        // --- 1. SENSE ---
        vehicle_sim::VehicleState egoState{
            egoVehicle.getVelocity(),
            egoVehicle.getAcceleration(),
            egoVehicle.getPosition()
        };

        vehicle_sim::Obstacle obstacle{
            targetX,
            targetSpeed,
            true
        };

        // --- 2. THINK (Behavior Tree Evaluation) ---
        double aebIntervention = collisionAvoidance.evaluate(egoState, obstacle);
        aebTriggered = (aebIntervention < 0.0);

        if (aebTriggered) {
            autoDrive = false;
            controlInput = -1.0; // Emergency braking override
            accActive = false;
        } else if (autoDrive) {
            accActive = true;
            controlInput = accController.computeControl(egoState, obstacle);
        } else {
            accActive = false;
            if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) {
                controlInput = 0.5;
            } else if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A)) {
                controlInput = -0.5;
            } else {
                controlInput = -0.1; // Coasting
            }
        }

        // --- 3. ACT ---
        egoVehicle.setControlInput(controlInput);
        egoVehicle.update(dt);

        float egoX = static_cast<float>(egoVehicle.getPosition());
        if (egoX + 60.0f >= targetX) {
            egoX = targetX - 60.0f;
            egoVehicle.setVelocity(targetSpeed);
        }

        // Telemetry
        double relativeDistance = obstacle.position - (egoX + 60.0f);
        if (relativeDistance < 0.0) relativeDistance = 0.0;
        double relativeSpeed = egoVehicle.getVelocity() - obstacle.velocity;
        double ttc = (relativeSpeed > 0.001) ? (relativeDistance / relativeSpeed) : 99.9;

        // --- 4. RENDER ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Track
        DrawRectangle(0, 280, 800, 100, LIGHTGRAY);
        DrawLine(0, 330, 800, 330, WHITE);

        // Target Vehicle
        DrawRectangle((int)targetX, 240, 50, 30, RED);
        DrawCircle((int)targetX + 12, 275, 7, BLACK);
        DrawCircle((int)targetX + 38, 275, 7, BLACK);

        // Sensor Beam
        if (egoVehicle.getVelocity() > 0 || aebTriggered) {
            Color beamColor = aebTriggered ? RED : (accActive ? BLUE : GREEN);
            float beamLength = (relativeDistance < sensorRange) ? static_cast<float>(relativeDistance) : sensorRange;
            DrawRectangle((int)(egoX + 60), 258, (int)beamLength, 10, Fade(beamColor, 0.3f));
            DrawLine((int)(egoX + 60), 263, (int)(egoX + 60 + beamLength), 263, beamColor);
        }

        // Ego Vehicle
        DrawRectangle((int)egoX, 240, 60, 30, DARKBLUE);
        DrawCircle((int)egoX + 15, 275, 8, BLACK);
        DrawCircle((int)egoX + 45, 275, 8, BLACK);

        // HUD
        DrawRectangle(10, 10, 460, 210, Fade(BLACK, 0.8f));
        Color statusColor = aebTriggered ? RED : (accActive ? BLUE : GREEN);
        const char* statusText = aebTriggered ? "AEB ACTIVE (EMERGENCY BRAKE)" : (accActive ? "ACC ACTIVE (ADAPTIVE CRUISE)" : "MANUAL CONTROL (CLICK TO ENABLE ACC)");
        
        DrawText(statusText, 25, 20, 15, statusColor);
        DrawText(TextFormat("Ego Speed: %.1f m/s | Target Speed: %.1f m/s", egoVehicle.getVelocity(), targetSpeed), 25, 45, 15, WHITE);
        DrawText(TextFormat("Closing Rate: %.1f m/s", relativeSpeed > 0 ? relativeSpeed : 0.0), 25, 70, 15, WHITE);
        DrawText(TextFormat("Relative Distance: %.1f m", relativeDistance), 25, 95, 15, WHITE);
        DrawText(TextFormat("TTC: %.2f s", ttc), 25, 120, 15, ttc < 2.0 ? ORANGE : WHITE);
        DrawText(TextFormat("Control Input: %.2f", controlInput), 25, 145, 15, WHITE);
        DrawText(TextFormat("Weight Transfer: %.1f N", egoVehicle.getWeightTransfer()), 25, 170, 15, WHITE);
        DrawText(TextFormat("Front Axle Load: %.1f N", egoVehicle.getFrontAxleLoad()), 25, 195, 15, WHITE);

        DrawText("Control: CLICK WINDOW for ACC | [UP/DOWN] Target Speed | [R] Reset", 15, 415, 14, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
