#pragma once

#include <Arduino.h>
#include <array>

namespace easy_drive {

class SeatSensorManager {
public:
    struct Config {
        std::array<uint8_t, 3> pin_assignments = {25, 26, 27}; // ESP32 GPIO pins for 3 seats
        unsigned long debounce_delay_ms = 50;                  // Debounce filter threshold
    };

    SeatSensorManager(const Config& config = Config()) : config_(config) {}

    void begin() {
        for (uint8_t pin : config_.pin_assignments) {
            // Active-low configuration with internal pull-up resistor
            pinMode(pin, INPUT_PULLUP);
        }
    }

    /**
     * @brief Checks occupancy of all 3 seats with debouncing.
     * @return true if all 3 seats are physically occupied.
     */
    bool areAllSeatsOccupied() const {
        for (uint8_t pin : config_.pin_assignments) {
            if (!isSeatOccupied(pin)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Returns the count of currently occupied seats (0 to 3).
     */
    uint8_t getOccupiedSeatCount() const {
        uint8_t count = 0;
        for (uint8_t pin : config_.pin_assignments) {
            if (isSeatOccupied(pin)) {
                count++;
            }
        }
        return count;
    }

private:
    bool isSeatOccupied(uint8_t pin) const {
        // Active-low: LOW means pressure applied (seated)
        int reading1 = digitalRead(pin);
        delayMicroseconds(100);
        int reading2 = digitalRead(pin);
        
        // Basic debounce check
        if (reading1 == reading2 && reading1 == LOW) {
            return true;
        }
        return false;
    }

    Config config_;
};

} // namespace easy_drive
