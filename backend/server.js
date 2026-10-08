/**
 * Easy Drive: Cloud API & IoT Backend Server (Zero-Dependency Native Node.js)
 * Features: Static file serving (Passenger app & Marshal dashboard), Anywhere QR booking, State Machine, Marshal Override, and Financial Ledger.
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const TransitService = require('./transit_service');

const transit = new TransitService();
let latestDispatchedTrip = null;

const server = http.createServer((req, res) => {
    const url = new URL(req.url, `http://${req.headers.host}`);
    
    const sendJson = (statusCode, data) => {
        res.writeHead(statusCode, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(data));
    };

    const parseBody = (callback) => {
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
            try {
                callback(JSON.parse(body));
            } catch (e) {
                sendJson(400, { success: false, message: "Invalid JSON body" });
            }
        });
    };

    // Static Frontend Serving
    if (url.pathname === '/' || url.pathname === '/index.html') {
        fs.readFile(path.join(__dirname, '../public/index.html'), (err, data) => {
            if (err) {
                res.writeHead(404);
                return res.end('Passenger app not found');
            }
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(data);
        });
        return;
    }

    if (url.pathname === '/marshal' || url.pathname === '/marshal.html') {
        fs.readFile(path.join(__dirname, '../public/marshal.html'), (err, data) => {
            if (err) {
                res.writeHead(404);
                return res.end('Marshal dashboard not found');
            }
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(data);
        });
        return;
    }

    // API Routes
    if (url.pathname === '/api/stations' && req.method === 'GET') {
        const stationsList = Array.from(transit.stations.values());
        return sendJson(200, { success: true, stations: stationsList });
    }

    if (url.pathname === '/api/book' && req.method === 'POST') {
        return parseBody(data => {
            const { passengerId, origin, destination, paymentType, marshalId, fare } = data;
            if (!passengerId || !origin || !destination) {
                return res.status(400).json({ success: false, message: "Missing passengerId, origin, or destination." });
            }

            const result = transit.bookSeat(
                passengerId, 
                parseInt(origin), 
                parseInt(destination), 
                paymentType || 'DIGITAL', 
                marshalId || null, 
                fare ? parseFloat(fare) : undefined
            );

            if (result.success && result.dispatchTriggered) {
                latestDispatchedTrip = transit.activeTrips[transit.activeTrips.length - 1];
            }
            return sendJson(200, result);
        });
    }

    if (url.pathname === '/api/marshal/override' && req.method === 'POST') {
        return parseBody(data => {
            const { action } = data; // 'DISPATCH' or 'RECALL'
            const success = transit.marshalOverride(action);
            if (success && action === 'DISPATCH') {
                latestDispatchedTrip = transit.activeTrips[transit.activeTrips.length - 1];
            }
            return sendJson(200, { success, vehicleState: transit.vehicleState });
        });
    }

    if (url.pathname === '/api/financial/ledger' && req.method === 'GET') {
        return sendJson(200, { success: true, ledger: transit.getFinancialLedgerSummary() });
    }

    if (url.pathname === '/api/queue/status' && req.method === 'GET') {
        return sendJson(200, { success: true, status: transit.getQueueStatus() });
    }

    if (url.pathname === '/api/vehicle/dispatch-status' && req.method === 'GET') {
        if (latestDispatchedTrip) {
            const tripToSend = latestDispatchedTrip;
            latestDispatchedTrip = null; // Acknowledge
            return sendJson(200, { success: true, dispatched: true, trip: tripToSend });
        } else {
            return sendJson(200, { success: true, dispatched: false });
        }
    }

    return sendJson(404, { success: false, message: "Endpoint not found" });
});

server.on('error', (err) => {
    console.error('[EASY DRIVE CLOUD ERROR]', err);
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, '0.0.0.0', () => {
    console.log(`[EASY DRIVE CLOUD] Backend server running on http://localhost:${PORT}`);
});

module.exports = server;
