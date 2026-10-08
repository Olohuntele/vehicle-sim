# Easy Drive Cloud API Reference

Base URL: `http://localhost:3000`

The Easy Drive backend is built with native Node.js, providing zero-dependency REST endpoints for passenger booking, station management, transport marshal overrides, and financial split ledgers.

---

## Summary of Endpoints

| Method | Endpoint | Description |
| :--- | :--- | :--- |
| `GET` | `/api/stations` | Retrieve list of all 52 campus stations. |
| `POST` | `/api/book` | Book a seat via Anywhere QR code (Stations #1–#52). |
| `POST` | `/api/marshal/override` | Execute transport marshal manual override (`DISPATCH` / `RECALL`). |
| `GET` | `/api/financial/ledger` | Retrieve revenue split ledger (75% university / 25% developer). |
| `GET` | `/api/queue/status` | Get active passenger queue and vehicle state machine status. |
| `GET` | `/api/vehicle/dispatch-status` | Polling endpoint for active trip dispatches. |

---

## Endpoint Details

### 1. Get Stations
Retrieve all 52 mapped campus stations with coordinates and station names.

- **URL:** `/api/stations`
- **Method:** `GET`
- **Response:**
  ```json
  {
    "success": true,
    "stations": [
      { "id": 1, "name": "Main Gate Station", "lat": 11.9845, "lon": 8.5342 },
      { "id": 2, "name": "Faculty of Engineering", "lat": 11.9852, "lon": 8.5358 }
    ]
  }
  ```
- **Example cURL:**
  ```bash
  curl -X GET http://localhost:3000/api/stations
  ```

---

### 2. Book Seat (Anywhere QR Booking)
Book a transport slot from origin to destination station. Automatically calculates distance-based fares, queue priority, and revenue splits.

- **URL:** `/api/book`
- **Method:** `POST`
- **Headers:** `Content-Type: application/json`
- **Request Body:**
  ```json
  {
    "passengerId": "STUDENT-1042",
    "origin": 1,
    "destination": 15,
    "paymentType": "DIGITAL",
    "fare": 200.00
  }
  ```
- **Response:**
  ```json
  {
    "success": true,
    "message": "Booking confirmed and added to queue.",
    "etaMinutes": 6,
    "dispatchTriggered": false,
    "financialSplit": {
      "totalFare": 200.00,
      "universityShare": 150.00,
      "developerShare": 50.00
    }
  }
  ```
- **Example cURL:**
  ```bash
  curl -X POST http://localhost:3000/api/book \
    -H "Content-Type: application/json \
    -d '{"passengerId": "STUDENT-1042", "origin": 1, "destination": 15, "paymentType": "DIGITAL", "fare": 200}'
  ```

---

### 3. Marshal Override
Allows authorized transport marshals to manually override vehicle state machine timeouts (e.g. force dispatch or recall).

- **URL:** `/api/marshal/override`
- **Method:** `POST`
- **Headers:** `Content-Type: application/json`
- **Request Body:**
  ```json
  {
    "action": "DISPATCH",
    "marshalId": "MARSHAL-007"
  }
  ```
- **Response:**
  ```json
  {
    "success": true,
    "vehicleState": "TRIP_IN_PROGRESS"
  }
  ```
- **Example cURL:**
  ```bash
  curl -X POST http://localhost:3000/api/marshal/override \
    -H "Content-Type: application/json" \
    -d '{"action": "DISPATCH"}'
  ```

---

### 4. Financial Ledger Summary
Retrieves cumulative fare collections and the automated 75/25 revenue split breakdown.

- **URL:** `/api/financial/ledger`
- **Method:** `GET`
- **Response:**
  ```json
  {
    "success": true,
    "ledger": {
      "totalRevenue": 12500.00,
      "universityTotal": 9375.00,
      "developerRetainerTotal": 3125.00,
      "transactionCount": 62
    }
  }
  ```
- **Example cURL:**
  ```bash
  curl -X GET http://localhost:3000/api/financial/ledger
  ```

---

### 5. Queue Status & Vehicle State
Returns the current queue depth, vehicle state (`WAITING_AT_STATION` or `TRIP_IN_PROGRESS`), and active passenger manifest.

- **URL:** `/api/queue/status`
- **Method:** `GET`
- **Response:**
  ```json
  {
    "success": true,
    "status": {
      "vehicleState": "WAITING_AT_STATION",
      "queueLength": 4,
      "passengersOnboard": 3
    }
  }
  ```
- **Example cURL:**
  ```bash
  curl -X GET http://localhost:3000/api/queue/status
  ```

---

### 6. Vehicle Dispatch Polling
Used by the simulation or IoT fleet controller to check for pending dispatch orders.

- **URL:** `/api/vehicle/dispatch-status`
- **Method:** `GET`
- **Response:**
  ```json
  {
    "success": true,
    "dispatched": true,
    "trip": {
      "origin": 1,
      "destination": 15,
      "passengerIds": ["STUDENT-1001", "STUDENT-1002", "STUDENT-1003"]
    }
  }
  ```
- **Example cURL:**
  ```bash
  curl -X GET http://localhost:3000/api/vehicle/dispatch-status
  ```
