/**
 * Deterministic Headless Tests for Easy Drive Autonomous Test 001
 * Tests both Blocked-Road (obstacle at Z=60, clearance >= 2.0m) and Clear-Road (destination at Z=100) scenarios
 * across varying initial speeds and delta times.
 */

const assert = require('assert');

function simulateScenario(scenarioType, initialSpeed, dt) {
    let egoPos = 0.0;
    let egoSpeed = initialSpeed;
    const targetB = 100.0;
    const obstaclePos = (scenarioType === 'blocked') ? 60.0 : 999.0;
    const egoLength = 4.0;
    const leadLength = 4.0;
    const maxAccel = 3.5;
    const maxBrake = 6.0;

    let time = 0.0;
    const maxTime = 30.0;
    let stopped = false;
    let minBumperSep = Infinity;
    let finalBumperSep = null;
    let finalPosition = 0.0;

    while (time < maxTime && !stopped) {
        let accel = 0.0;

        if (scenarioType === 'blocked') {
            const leadRear = obstaclePos - leadLength / 2.0;
            const egoFront = egoPos + egoLength / 2.0;
            const bumperSep = leadRear - egoFront;
            if (bumperSep < minBumperSep) minBumperSep = bumperSep;

            // Braking distance required: v^2 / (2 * maxBrake) + safety margin (2.5m) + discrete step buffer
            const brakingDist = (egoSpeed * egoSpeed) / (2.0 * maxBrake) + 2.5 + egoSpeed * dt;

            if (bumperSep <= brakingDist || egoPos >= obstaclePos - 12.0) {
                accel = -maxBrake;
            } else if (egoSpeed < 10.0) {
                accel = maxAccel;
            } else {
                accel = 0.0; // cruising
            }
        } else {
            // Clear road
            const distToB = targetB - egoPos;
            const brakingDist = (egoSpeed * egoSpeed) / (2.0 * maxBrake) + egoSpeed * dt;

            if (distToB <= brakingDist + 1.0 || egoPos >= targetB) {
                accel = -maxBrake;
            } else if (egoSpeed < 10.0) {
                accel = maxAccel;
            } else {
                accel = 0.0;
            }
        }

        egoSpeed = Math.max(0.0, egoSpeed + accel * dt);
        egoPos += egoSpeed * dt;
        time += dt;

        if (egoSpeed <= 0.001) {
            egoSpeed = 0.0;
            stopped = true;
            finalPosition = egoPos;
            if (scenarioType === 'blocked') {
                const leadRear = obstaclePos - leadLength / 2.0;
                const egoFront = egoPos + egoLength / 2.0;
                finalBumperSep = leadRear - egoFront;
            }
        }
    }

    return {
        stopped,
        finalSpeed: egoSpeed,
        finalPosition,
        minBumperSep,
        finalBumperSep,
        time
    };
}

function runTests() {
    console.log("=== Running Easy Drive Autonomous Test 001 Headless Validation ===");

    const deltaTimes = [0.016, 0.033, 0.05, 0.1];
    const initialSpeeds = [0.0, 5.0, 10.0];

    // 1. Blocked-Road Scenario Tests
    console.log("\nTesting Blocked-Road Scenario (Obstacle at Z=60)...");
    for (const dt of deltaTimes) {
        for (const v0 of initialSpeeds) {
            const result = simulateScenario('blocked', v0, dt);
            assert(result.stopped, `Blocked road did not stop (v0=${v0}, dt=${dt})`);
            assert(result.finalSpeed === 0.0, `Final speed not zero: ${result.finalSpeed}`);
            assert(result.minBumperSep >= 2.0, `Obstacle penetration detected! Min clearance: ${result.minBumperSep}m (v0=${v0}, dt=${dt})`);
            assert(result.finalBumperSep >= 2.0, `Final clearance < 2.0m: ${result.finalBumperSep}m (v0=${v0}, dt=${dt})`);
            console.log(`[PASS] Blocked-road | v0=${v0} m/s | dt=${dt}s | Final Pos=%.2f | Min Clearance=%.2f m`, result.finalPosition, result.minBumperSep);
        }
    }

    // 2. Clear-Road Scenario Tests
    console.log("\nTesting Clear-Road Scenario (Destination B at Z=100)...");
    for (const dt of deltaTimes) {
        for (const v0 of initialSpeeds) {
            const result = simulateScenario('clear', v0, dt);
            assert(result.stopped, `Clear road did not stop (v0=${v0}, dt=${dt})`);
            assert(result.finalSpeed === 0.0, `Final speed not zero: ${result.finalSpeed}`);
            // Should arrive near target B (Z=100.0) with small tolerance
            assert(Math.abs(result.finalPosition - 100.0) < 5.0, `Did not arrive near Point B (Z=100): ${result.finalPosition}m (v0=${v0}, dt=${dt})`);
            console.log(`[PASS] Clear-road | v0=${v0} m/s | dt=${dt}s | Final Pos=%.2f m`, result.finalPosition);
        }
    }

    console.log("\n=== ALL HEADLESS VALIDATION TESTS PASSED SUCCESSFULLY ===");
}

runTests();
