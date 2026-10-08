#pragma once

#include "firmware/esp32_seat_sensor/SeatSensor.hpp"
#include "firmware/esp32_can/CanBusManager.hpp"

namespace easy_drive {

class VehicleMainController {
public:
    struct Config {
        SeatSensorManager::Config seat_config;
        CanBusManager::Config can_config;
    };

    VehicleMainController(const Config& config = Config())
        : seat_manager_(config.seat_config), can_manager_(config.can_config), trip_dispatched_(false), motor_unlocked_(false) {}

    bool begin() {
        seat_manager_.begin();
        if (!can_manager_.begin()) {
            return false;
        }
        return true;
    }

    /**
     * @brief Main control loop execution step.
     * Combines cloud trip dispatch authorization with physical seat pressure sensor verification.
     */
    void update(bool cloud_trip_dispatched) {
        trip_dispatched_ = cloud_trip_dispatched;

        // Check physical seat occupancy
        bool all_seats_occupied = seat_manager_.areAllSeatsOccupied();

        // Safety Condition: Motor unlocks ONLY IF cloud trip is dispatched AND all 3 seats are physically occupied
        bool should_unlock_motor = trip_dispatched_ && all_seats_occupied;

        if (should_unlock_motor != motor_unlocked_) {
            motor_unlocked_ = should_unlock_motor;
            can_manager_.sendMotorEnableCommand(motor_unlocked_);
        }

        // Poll BMS telemetry
        CanBusManager::BMSTelemetry telemetry;
        if (can_manager_.pollTelemetry(telemetry)) {
            // Handle battery telemetry (e.g. check low voltage / temperature limits)
            if (telemetry.valid && telemetry.state_of_charge < 15.0f) {
                // Low battery warning -> force motor lock for safety
                can_manager_.sendMotorEnableCommand(false);
                motor_unlocked_ = false;
            }
        }
    }

    bool isMotorUnlocked() const { return motor_unlocked_; }
    uint8_t getOccupiedSeats() const { return seat_manager_.getOccupiedSeatCount(); }

private:
    SeatSensorManager seat_manager_;
    CanBusManager can_manager_;
    bool trip_dispatched_;
    bool motor_unlocked_;
};

} // namespace easy_drive
