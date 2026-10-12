#include "raylib.h"
#include "Vehicle.hpp"
#include "vehicle_core.hpp"
#include <iostream>
#include <cmath>

int main() {
    const int screenWidth = 800;
    const int screenHeight = 450;
    
    InitWindow(screenWidth, screenHeight, "Autonomous Vehicle Sim - Behavior Tree & ACC");
    SetTargetFPS(60);

    // Instantiate Ego Vehicle and Core Controller
    Vehicle egoVehicle(0.0);
    vehicle_sim::CollisionAvoidance::Config caConfig{2.0, 5.0, -8.0};
    vehicle_sim::ACCController::Config accConfig{8.0, 1.5, 0.5, 0.3};
    vehicle_sim::VehicleCoreController coreController(caConfig, accConfig);

    // Moving Target Vehicle (Red Obstacle) in meters
    double targetPosition = 50.0;
    double targetSpeed = 2.0;
    const double sensorRange = 35.0; // meters
    const float scale = 10.0f; // pixels per meter
    
    bool autoDrive = false;
    const double dt = 1.0 / 60.0;

    while (!WindowShouldClose()) {
        // --- Controls ---
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyPressed(KEY_SPACE)) {
            autoDrive = !autoDrive;
        }

        // Reset Scenario
        if (IsKeyPressed(KEY_R)) {
            egoVehicle.reset(0.0);
            targetPosition = 50.0;
            targetSpeed = 2.0;
            autoDrive = false;
        }

        // Control Target Vehicle Speed
        if (IsKeyDown(KEY_UP)) targetSpeed += 0.05;
        if (IsKeyDown(KEY_DOWN)) targetSpeed -= 0.05;
        if (targetSpeed < 0.0) targetSpeed = 0.0;
        if (targetSpeed > 15.0) targetSpeed = 15.0;

        // Move Target Vehicle (meters)
        targetPosition += targetSpeed * dt;

        // --- 1. SENSE & EVALUATE (via vehicle_core) ---
        vehicle_sim::CoreInput input;
        input.ego_state = { egoVehicle.getVelocity(), egoVehicle.getAcceleration(), egoVehicle.getPosition() };
        input.obstacle = { targetPosition, targetSpeed, true };
        input.auto_drive = autoDrive;
        input.key_right_pressed = IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D);
        input.key_left_pressed = IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A);
        input.vehicle_length = 6.0; // ego box width/length representation in sim
        input.obstacle_length = 5.0; // target box width/length representation

        vehicle_sim::CoreOutput output = coreController.evaluate(input);

        // --- 2. ACT ---
        egoVehicle.setControlInput(output.control_input);
        egoVehicle.update(dt);

        // --- 3. RENDER ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Track
        DrawRectangle(0, 280, 800, 100, LIGHTGRAY);
        DrawLine(0, 330, 800, 330, WHITE);

        // Convert positions to screen pixels for rendering
        float egoScreenX = 100.0f;
        float targetScreenX = egoScreenX + static_cast<float>(targetPosition - egoVehicle.getPosition()) * scale;
        if (targetScreenX > 750.0f) targetScreenX = 750.0f;
        if (targetScreenX < egoScreenX + 20.0f) targetScreenX = egoScreenX + 20.0f;

        // Target Vehicle
        DrawRectangle((int)targetScreenX, 240, 50, 30, RED);
        DrawCircle((int)targetScreenX + 12, 275, 7, BLACK);
        DrawCircle((int)targetScreenX + 38, 275, 7, BLACK);

        // Sensor Beam
        if (egoVehicle.getVelocity() > 0 || output.aeb_triggered) {
            Color beamColor = output.aeb_triggered ? RED : (output.acc_active ? BLUE : GREEN);
            float beamLengthMeters = (output.relative_distance < sensorRange) ? static_cast<float>(output.relative_distance) : static_cast<float>(sensorRange);
            float beamLengthPixels = beamLengthMeters * scale;
            DrawRectangle((int)(egoScreenX + 60), 258, (int)beamLengthPixels, 10, Fade(beamColor, 0.3f));
            DrawLine((int)(egoScreenX + 60), 263, (int)(egoScreenX + 60 + beamLengthPixels), 263, beamColor);
        }

        // Ego Vehicle
        DrawRectangle((int)egoScreenX, 240, 60, 30, DARKBLUE);
        DrawCircle((int)egoScreenX + 15, 275, 8, BLACK);
        DrawCircle((int)egoScreenX + 45, 275, 8, BLACK);

        // HUD
        DrawRectangle(10, 10, 460, 225, Fade(BLACK, 0.8f));
        Color statusColor = output.aeb_triggered ? RED : (output.acc_active ? BLUE : GREEN);
        const char* statusText = output.aeb_triggered ? "AEB ACTIVE (EMERGENCY BRAKE)" : (output.acc_active ? "ACC ACTIVE (ADAPTIVE CRUISE)" : "MANUAL CONTROL (CLICK TO ENABLE ACC)");
        
        DrawText(statusText, 25, 20, 15, statusColor);
        DrawText(TextFormat("Ego Speed: %.1f m/s | Target Speed: %.1f m/s", egoVehicle.getVelocity(), targetSpeed), 25, 45, 15, WHITE);
        DrawText(TextFormat("Closing Rate: %.1f m/s", output.relative_speed > 0 ? output.relative_speed : 0.0), 25, 70, 15, WHITE);
        DrawText(TextFormat("Relative Distance: %.1f m", output.relative_distance), 25, 95, 15, WHITE);
        DrawText(TextFormat("TTC: %.2f s", output.ttc), 25, 120, 15, output.ttc < 2.0 ? ORANGE : WHITE);
        DrawText(TextFormat("Bumper Clearance: %.1f m", output.bumper_clearance), 25, 145, 15, WHITE);
        DrawText(TextFormat("Control Input: %.2f", output.control_input), 25, 170, 15, WHITE);
        DrawText(TextFormat("Weight Transfer: %.1f N", egoVehicle.getWeightTransfer()), 25, 195, 15, WHITE);

        DrawText("Control: CLICK WINDOW for ACC | [UP/DOWN] Target Speed | [R] Reset", 15, 415, 14, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
