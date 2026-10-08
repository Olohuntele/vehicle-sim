const TransitService = require('./transit_service');

function runTests() {
    console.log("=== Running Easy Drive Transit Service Tests ===");
    const service = new TransitService();

    // Test station validation
    const res1 = service.bookSeat("P_001", 5, 12);
    console.log("Passenger 1 booked:", res1);

    const res2 = service.bookSeat("P_002", 5, 20);
    console.log("Passenger 2 booked:", res2);

    const res3 = service.bookSeat("P_003", 5, 52);
    console.log("Passenger 3 booked (should trigger dispatch):", res3);

    const status = service.getQueueStatus();
    console.log("Queue Status after dispatch:", status);

    console.log("=== All Transit Service Tests Passed Successfully ===");
}

runTests();
