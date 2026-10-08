/**
 * Easy Drive: End-to-End System Integration Test
 * Simulates student QR bookings and ESP32 vehicle polling against backend/server.js.
 */

const http = require('http');
const app = require('./server');

const PORT = 3001; // Test port
const server = http.createServer(app);

server.listen(PORT, async () => {
    console.log(`[TEST] Integration test server started on port ${PORT}`);

    const makeRequest = (method, path, data = null) => {
        return new Promise((resolve, reject) => {
            const dataStr = data ? JSON.stringify(data) : '';
            const req = http.request({
                hostname: 'localhost',
                port: PORT,
                path: path,
                method: method,
                headers: {
                    'Content-Type': 'application/json',
                    'Content-Length': dataStr.length
                }
            }, (res) => {
                let body = '';
                res.on('data', chunk => body += chunk);
                res.on('end', () => resolve(JSON.parse(body)));
            });
            req.on('error', reject);
            if (dataStr) req.write(dataStr);
            req.end();
        });
    };

    try {
        // 1. Fetch Stations (#1 to #52)
        console.log("\n--- Testing GET /api/stations ---");
        const stationsRes = await makeRequest('GET', '/api/stations');
        console.log(`Stations loaded: ${stationsRes.stations.length} stations found (Expected 52).`);
        assert(stationsRes.stations.length === 52);

        // 2. Book Passenger 1
        console.log("\n--- Testing Passenger 1 Booking ---");
        const book1 = await makeRequest('POST', '/api/book', { passengerId: 'STUDENT_01', origin: 5, destination: 12 });
        console.log("Booking 1 response:", book1);

        // 3. Book Passenger 2
        console.log("\n--- Testing Passenger 2 Booking ---");
        const book2 = await makeRequest('POST', '/api/book', { passengerId: 'STUDENT_02', origin: 5, destination: 20 });
        console.log("Booking 2 response:", book2);

        // 4. Book Passenger 3 (Triggers Wait-Until-Full Dispatch)
        console.log("\n--- Testing Passenger 3 Booking (Triggering Dispatch) ---");
        const book3 = await makeRequest('POST', '/api/book', { passengerId: 'STUDENT_03', origin: 5, destination: 52 });
        console.log("Booking 3 response:", book3);
        assert(book3.dispatchTriggered === true);

        // 5. Simulate ESP32 Polling Dispatch Status
        console.log("\n--- Testing ESP32 Polling GET /api/vehicle/dispatch-status ---");
        const espPoll = await makeRequest('GET', '/api/vehicle/dispatch-status');
        console.log("ESP32 Poll response:", espPoll);
        assert(espPoll.dispatched === true);
        assert(espPoll.trip.passengers.length === 3);

        console.log("\n=== ALL END-TO-END INTEGRATION TESTS PASSED SUCCESSFULLY ===");
    } catch (err) {
        console.error("Integration test failed:", err);
    } finally {
        server.close();
    }
});

function assert(condition) {
    if (!condition) {
        throw new Error("Assertion failed!");
    }
}
