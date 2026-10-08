# Project State Log: Easy Drive (Autonomous Campus Transport Fleet)

## Active Baseline & Verified Features
1. **Simulation & Core Physics (`vehicle-sim`)**: 
   - Kinematics, weight transfer, friction limits, AEB, ACC, and CTest unit tests fully operational and passing.
2. **Financial & Revenue Model**: 
   - Automated **25%/75% split** implemented.
   - Digital payment split payloads and cash proxy ledger verified.
3. **Operational State Machine**: 
   - `WAITING_AT_STATION` and `TRIP_IN_PROGRESS` logic enforced.
   - **Anywhere QR Booking** from stations #1–#52 with dynamic ETA verified.
4. **Cloud Backend & API (`backend/`)**: 
   - Native Node.js REST API serving all endpoints and static frontend files.
5. **Frontend Web UIs (`public/`)**: 
   - **Passenger Booking App** and **Marshal Dashboard** functional and tested against the live backend.
6. **ESP32 Firmware (`firmware/`)**: 
   - Seat sensor management, CAN bus motor/BMS communication, and HIL safety interlock logic completed.

## Next Development Phase
- **Milestone 5: Physical Deployment & Field Testing**
  - Finalize hardware wiring schematics and assembly of the electric trike prototype.
  - Deployment of QR station markers at ADUSTW Wudil.
  - On-site pilot testing with campus marshals.
