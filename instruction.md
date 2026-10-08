# Easy Drive Project Mandate: Autonomous Campus Transport Fleet

## 1. Project Overview
**Easy Drive** is an autonomous campus transport fleet designed for university environments in Nigeria (Target: ADUSTW Wudil). It operates on a B2G institutional model where the university owns the fleet and generates revenue.

## 2. Core Architecture & Constraints
- **Simulation (`vehicle-sim`):** C++17/Raylib/CMake. Must maintain decoupled physical dynamic models (friction limits, aerodynamic drag, TTC) from rendering.
- **Firmware (`firmware/`):** ESP32-based. Handles 3x seat pressure sensors (active-low), MCP2515 CAN bus (500kbps) for motor/BMS communication, and safety interlocks.
- **Backend (`backend/`):** Native Node.js REST API. Manages 52 campus stations, QR booking queues, and vehicle state machine logic.
- **Financial Model:** Mandatory **25% developer maintenance retainer** and **75% university share** split for all ticket revenue (NGN).
- **Operational Logic:**
  - `WAITING_AT_STATION`: Requires 3 active seat sensors to depart.
  - `TRIP_IN_PROGRESS`: Ignores intermediate seat drops until trip completion.
  - **Marshal Override:** Manual dashboard buttons take precedence over automated timeouts.

## 3. Code Standards
- **Modularity:** Keep physics, IoT firmware, and backend logic strictly separated.
- **Safety First:** Vehicle propulsion must remain locked via CAN bus until both cloud dispatch and physical seating are verified.
- **Financial Integrity:** All booking transactions must be logged in the operational ledger with appropriate revenue splits.
