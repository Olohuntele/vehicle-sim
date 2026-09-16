#include "raylib.h"
#include <iostream>
#include <cmath>

int main() {
    const int screenWidth = 800;
    const int screenHeight = 450;
    
    InitWindow(screenWidth, screenHeight, "AEB Simulation - Interactive Telemetry");
    SetTargetFPS(60);

    float carX = 50.0f;
    float carSpeed = 0.0f;
    const float maxSpeed = 12.0f;
    const float accel = 0.15f;
    const float aebBrake = 0.4f;
    
    const float obstacleX = 650.0f;
    const float sensorRange = 300.0f;
    bool aebTriggered = false;
    bool autoDrive = false;

    while (!WindowShouldClose()) {
        // --- Controls: Mouse Click or Keys ---
        // Click screen to toggle Auto-Drive, or press R to reset
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyPressed(KEY_SPACE)) {
            autoDrive = !autoDrive;
        }

        if (IsKeyPressed(KEY_R)) {
            carX = 50.0f;
            carSpeed = 0.0f;
            aebTriggered = false;
            autoDrive = false;
        }

        // --- Controller & Physics Logic ---
        float distanceToObstacle = obstacleX - (carX + 60.0f);
        if (distanceToObstacle < 0.0f) distanceToObstacle = 0.0f;

        float ttc = (carSpeed > 0.001f) ? (distanceToObstacle / (carSpeed * 60.0f)) : 99.9f;
        float stoppingDistance = (carSpeed * carSpeed) / (2.0f * aebBrake);

        // AEB Trigger Condition
        if (distanceToObstacle > 0.0f && distanceToObstacle <= stoppingDistance + 20.0f && carSpeed > 0.5f) {
            aebTriggered = true;
            autoDrive = false;
        }

        // Drive / Brake Logic
        if (aebTriggered) {
            carSpeed -= aebBrake;
            if (carSpeed <= 0.0f) carSpeed = 0.0f;
        } else {
            if (autoDrive || IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) {
                carSpeed += accel;
            } else {
                carSpeed -= 0.05f; // Gradual slowdown when not accelerating
            }
            if (carSpeed > maxSpeed) carSpeed = maxSpeed;
            if (carSpeed < 0.0f) carSpeed = 0.0f;
        }

        carX += carSpeed;
        if (carX + 60.0f >= obstacleX) {
            carX = obstacleX - 60.0f;
            carSpeed = 0.0f;
        }

        // --- Rendering ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Track & Obstacle
        DrawRectangle(0, 280, 800, 100, LIGHTGRAY);
        DrawLine(0, 330, 800, 330, WHITE);
        DrawRectangle((int)obstacleX, 230, 25, 70, RED);

        // Sensor Field Beam
        if (carSpeed > 0 || aebTriggered) {
            Color beamColor = aebTriggered ? RED : (ttc < 2.0f ? ORANGE : GREEN);
            float beamLength = (distanceToObstacle < sensorRange) ? distanceToObstacle : sensorRange;
            DrawRectangle((int)(carX + 60), 258, (int)beamLength, 10, Fade(beamColor, 0.3f));
            DrawLine((int)(carX + 60), 263, (int)(carX + 60 + beamLength), 263, beamColor);
        }

        // Vehicle
        DrawRectangle((int)carX, 240, 60, 30, DARKBLUE);
        DrawCircle((int)carX + 15, 275, 8, BLACK);
        DrawCircle((int)carX + 45, 275, 8, BLACK);

        // HUD
        DrawRectangle(10, 10, 380, 150, Fade(BLACK, 0.8f));
        Color statusColor = aebTriggered ? RED : (ttc < 2.0f ? ORANGE : GREEN);
        const char* statusText = aebTriggered ? "AEB ACTIVE (EMERGENCY BRAKE)" : (autoDrive ? "DRIVING (CLICK TO PAUSE)" : "CLICK ANYWHERE TO DRIVE");
        
        DrawText(statusText, 25, 25, 16, statusColor);
        DrawText(TextFormat("Speed: %.1f px/f (%.1f km/h eq)", carSpeed, carSpeed * 10.0f), 25, 55, 16, WHITE);
        DrawText(TextFormat("Distance: %.1f px", distanceToObstacle), 25, 80, 16, WHITE);
        DrawText(TextFormat("TTC: %.2f s", ttc), 25, 105, 16, ttc < 2.0f ? ORANGE : WHITE);
        DrawText(TextFormat("Stopping Dist: %.1f px", stoppingDistance), 25, 130, 16, WHITE);

        DrawText("Control: CLICK WINDOW to start/pause car drive | [R] Reset", 20, 410, 15, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
