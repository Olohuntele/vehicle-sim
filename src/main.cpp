#include "raylib.h"
#include <iostream>
#include <cmath>

int main() {
    const int screenWidth = 800;
    const int screenHeight = 450;
    
    InitWindow(screenWidth, screenHeight, "AEB Simulation - Dynamic Target Tracking");
    SetTargetFPS(60);

    // Ego Vehicle (Blue)
    float egoX = 50.0f;
    float egoSpeed = 0.0f;
    const float maxEgoSpeed = 12.0f;
    const float egoAccel = 0.15f;
    const float aebBrake = 0.4f;
    
    // Moving Target Vehicle (Red Obstacle)
    float targetX = 500.0f;
    float targetSpeed = 2.0f; // Initial cruising speed
    const float sensorRange = 350.0f;
    
    bool aebTriggered = false;
    bool autoDrive = false;

    while (!WindowShouldClose()) {
        // --- Controls ---
        // Click screen to toggle Ego Vehicle Auto-Drive
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyPressed(KEY_SPACE)) {
            autoDrive = !autoDrive;
        }

        // Reset Scenario
        if (IsKeyPressed(KEY_R)) {
            egoX = 50.0f;
            egoSpeed = 0.0f;
            targetX = 500.0f;
            targetSpeed = 2.0f;
            aebTriggered = false;
            autoDrive = false;
        }

        // Control Target Vehicle Speed (UP = speed up, DOWN = slow down/stop)
        if (IsKeyDown(KEY_UP)) targetSpeed += 0.05f;
        if (IsKeyDown(KEY_DOWN)) targetSpeed -= 0.05f;
        if (targetSpeed < 0.0f) targetSpeed = 0.0f;
        if (targetSpeed > 8.0f) targetSpeed = 8.0f;

        // Move Target Vehicle
        targetX += targetSpeed;
        if (targetX > 750.0f) targetX = 750.0f; // Track boundary

        // --- Kinematics & Relative Telemetry ---
        float relativeDistance = targetX - (egoX + 60.0f);
        if (relativeDistance < 0.0f) relativeDistance = 0.0f;

        // Relative speed (closing rate)
        float relativeSpeed = egoSpeed - targetSpeed;
        
        // Calculate Time-To-Collision based on relative velocity
        float ttc = (relativeSpeed > 0.001f) ? (relativeDistance / (relativeSpeed * 60.0f)) : 99.9f;

        // Required Stopping Distance based on relative deceleration needed
        float stoppingDistance = (egoSpeed * egoSpeed) / (2.0f * aebBrake);

        // AEB Trigger Condition: Closing in fast on moving or stationary target
        if (relativeDistance > 0.0f && relativeDistance <= stoppingDistance + 25.0f && relativeSpeed > 0.5f) {
            aebTriggered = true;
            autoDrive = false;
        }

        // Ego Braking / Drive Logic
        if (aebTriggered) {
            egoSpeed -= aebBrake;
            if (egoSpeed <= 0.0f) egoSpeed = 0.0f;
        } else {
            if (autoDrive || IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) {
                egoSpeed += egoAccel;
            } else {
                egoSpeed -= 0.05f;
            }
            if (egoSpeed > maxEgoSpeed) egoSpeed = maxEgoSpeed;
            if (egoSpeed < 0.0f) egoSpeed = 0.0f;
        }

        // Update Ego Position
        egoX += egoSpeed;
        if (egoX + 60.0f >= targetX) {
            egoX = targetX - 60.0f; // Bumper contact boundary
            egoSpeed = targetSpeed;
        }

        // --- Visual Rendering ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Track / Environment
        DrawRectangle(0, 280, 800, 100, LIGHTGRAY);
        DrawLine(0, 330, 800, 330, WHITE);

        // Moving Target Vehicle (Red)
        DrawRectangle((int)targetX, 240, 50, 30, RED);
        DrawCircle((int)targetX + 12, 275, 7, BLACK);
        DrawCircle((int)targetX + 38, 275, 7, BLACK);

        // Sensor Field Beam
        if (egoSpeed > 0 || aebTriggered) {
            Color beamColor = aebTriggered ? RED : (ttc < 2.5f ? ORANGE : GREEN);
            float beamLength = (relativeDistance < sensorRange) ? relativeDistance : sensorRange;
            DrawRectangle((int)(egoX + 60), 258, (int)beamLength, 10, Fade(beamColor, 0.3f));
            DrawLine((int)(egoX + 60), 263, (int)(egoX + 60 + beamLength), 263, beamColor);
        }

        // Ego Vehicle (Blue)
        DrawRectangle((int)egoX, 240, 60, 30, DARKBLUE);
        DrawCircle((int)egoX + 15, 275, 8, BLACK);
        DrawCircle((int)egoX + 45, 275, 8, BLACK);

        // Telemetry HUD
        DrawRectangle(10, 10, 420, 160, Fade(BLACK, 0.8f));
        Color statusColor = aebTriggered ? RED : (ttc < 2.5f ? ORANGE : GREEN);
        const char* statusText = aebTriggered ? "AEB ACTIVE (MOVING TARGET DETECTED)" : (autoDrive ? "DRIVING (CLICK TO PAUSE)" : "CLICK ANYWHERE TO DRIVE");
        
        DrawText(statusText, 25, 20, 15, statusColor);
        DrawText(TextFormat("Ego Speed: %.1f px/f | Target Speed: %.1f px/f", egoSpeed, targetSpeed), 25, 45, 15, WHITE);
        DrawText(TextFormat("Closing Rate (Rel Speed): %.1f px/f", relativeSpeed > 0 ? relativeSpeed : 0.0f), 25, 70, 15, WHITE);
        DrawText(TextFormat("Relative Distance: %.1f px", relativeDistance), 25, 95, 15, WHITE);
        DrawText(TextFormat("TTC: %.2f s", ttc), 25, 120, 15, ttc < 2.5f ? ORANGE : WHITE);
        DrawText(TextFormat("Req. Stopping Dist: %.1f px", stoppingDistance), 25, 145, 15, WHITE);

        DrawText("Control: CLICK WINDOW to start ego car | [UP/DOWN] Change target speed | [R] Reset", 15, 415, 14, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
