# Easy Drive Hardware & ESP32 Firmware Reference

The **Easy Drive** vehicle embedded subsystem is powered by an **ESP32 microcontroller**, interfacing with analog seat pressure sensors, an MCP2515 CAN bus controller (500kbps), and motor/BMS controllers.

---

## 🔌 ESP32 Pinout & Wiring Connections

| Component | ESP32 GPIO Pin | Protocol / Mode | Description |
| :--- | :--- | :--- | :--- |
| **Seat Sensor 1** | `GPIO 25` | `INPUT_PULLUP` (Active-Low) | Pressure mat under Driver / Front seat |
| **Seat Sensor 2** | `GPIO 26` | `INPUT_PULLUP` (Active-Low) | Pressure mat under Rear Passenger Left |
| **Seat Sensor 3** | `GPIO 27` | `INPUT_PULLUP` (Active-Low) | Pressure mat under Rear Passenger Right |
| **MCP2515 CS** | `GPIO 5` | SPI Chip Select | CAN Controller SPI Select |
| **MCP2515 SCK** | `GPIO 18` | SPI Clock | SPI Bus Clock |
| **MCP2515 MISO** | `GPIO 19` | SPI MISO | Master In Slave Out |
| **MCP2515 MOSI** | `GPIO 23` | SPI MOSI | Master Out Slave In |
| **MCP2515 INT** | `GPIO 4` | Digital Input | CAN Interrupt (Optional) |

---

## 🛡️ Hardware-in-the-Loop (HIL) Safety Interlock

The vehicle propulsion motor remains **electrically locked** by default. Motor unlock and vehicle dispatch require meeting two simultaneous safety gates:

1. **Cloud Dispatch Authorization:** The cloud backend (`backend/server.js`) confirms all passenger bookings and issues a trip dispatch command.
2. **Physical Seating Verification:** All **3 seat pressure sensors** must register occupancy (active-low signal `LOW`).

```mermaid
graph TD
    A[Cloud Dispatch Signal] --> C{Safety Gate}
    B[3x Seat Sensors Occupied] --> C
    C -->|Both Verified| D[CAN Bus: Unlock Motor (0x101 = 0x01)]
    C -->|Unverified / Low Battery| E[CAN Bus: Lock Motor (0x101 = 0x00)]
```

---

## 🚗 CAN Bus Communication (MCP2515 - 500 kbps)

- **Baud Rate:** 500 kbps (Standard Automotive CAN)
- **Clock Crystal:** 8 MHz
- **Motor Control Frame (`CAN ID: 0x101`):**
  - Data Byte 0: `0x01` (Enable Motor / Unlock) or `0x00` (Disable Motor / Lock).
- **BMS Telemetry Frame (`CAN ID: 0x201`):**
  - Bytes 0–1: State of Charge (SOC %, scaled by 10).
  - Bytes 2–3: Pack Voltage (Volts, scaled by 10).
  - Bytes 4–5: Pack Temperature (°Celsius, scaled by 10).
  - *Safety Rule:* If SOC drops below 15.0%, the firmware automatically overrides and triggers a motor lock.

---

## 📂 Firmware Module Structure

- **`firmware/esp32_seat_sensor/SeatSensor.hpp`**: Manages seat pressure sensor input polling with debounce filtering.
- **`firmware/esp32_can/CanBusManager.hpp`**: Handles MCP2515 SPI initialization, CAN message transmission, and BMS telemetry parsing.
- **`firmware/esp32_integration/MainController.hpp`**: Core control loop (`VehicleMainController`) executing safety interlock logic between cloud state and physical sensors.
