#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

namespace easy_drive {

class CanBusManager {
public:
    struct Config {
        uint8_t cs_pin = 5;          // SPI Chip Select pin for MCP2515 on ESP32
        MCP2515::Clock_t clock = MCP2515::CLOCK_8MHZ;
        MCP2515::Speed_t speed = MCP2515::CAN_500KBPS; // Standard automotive CAN baud rate
    };

    struct BMSTelemetry {
        float state_of_charge = 0.0; // Percentage (0-100%)
        float pack_voltage = 0.0;    // Volts
        float temperature = 0.0;     // Celsius
        bool valid = false;
    };

    CanBusManager(const Config& config = Config()) 
        : config_(config), mcp2515_(config_.cs_pin) {}

    bool begin() {
        SPI.begin();
        mcp2515_.reset();
        
        if (mcp2515_.setBitrate(config_.speed, config_.clock) != MCP2515::ERROR_OK) {
            return false;
        }
        
        if (mcp2515_.setNormalMode() != MCP2515::ERROR_OK) {
            return false;
        }
        
        return true;
    }

    /**
     * @brief Sends motor control command to enable or disable propulsion.
     * @param enable true to unlock motor (after wait-until-full confirmed), false to lock/stop.
     */
    bool sendMotorEnableCommand(bool enable) {
        struct can_frame frame;
        frame.can_id = 0x101; // Designated CAN ID for vehicle motor drive control
        frame.can_dlc = 1;
        frame.data[0] = enable ? 0x01 : 0x00; // 1 = Start permitted, 0 = Stop/Lock

        return (mcp2515_.sendMessage(&frame) == MCP2515::ERROR_OK);
    }

    /**
     * @brief Polls incoming CAN bus messages (e.g. BMS telemetry).
     */
    bool pollTelemetry(BMSTelemetry& out_telemetry) {
        struct can_frame frame;
        if (mcp2515_.readMessage(&frame) == MCP2515::ERROR_OK) {
            // Example CAN ID 0x201 for BMS Pack Status
            if (frame.can_id == 0x201 && frame.can_dlc >= 6) {
                // Parse State of Charge (bytes 0-1, scaled by 10)
                uint16_t soc_raw = (frame.data[0] << 8) | frame.data[1];
                out_telemetry.state_of_charge = soc_raw / 10.0f;

                // Parse Voltage (bytes 2-3, scaled by 10)
                uint16_t volt_raw = (frame.data[2] << 8) | frame.data[3];
                out_telemetry.pack_voltage = volt_raw / 10.0f;

                // Parse Temperature (bytes 4-5, offset by -40 or scaled)
                int16_t temp_raw = (frame.data[4] << 8) | frame.data[5];
                out_telemetry.temperature = temp_raw / 10.0f;

                out_telemetry.valid = true;
                return true;
            }
        }
        return false;
    }

private:
    Config config_;
    MCP2515 mcp2515_;
};

} // namespace easy_drive
