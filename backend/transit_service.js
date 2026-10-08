/**
 * Easy Drive: Cloud Transit Booking, State Machine & Financial Ledger
 * Features:
 * - Anywhere QR Booking (#1 to #52) with nearest vehicle ETA.
 * - State Machine: WAITING_AT_STATION vs TRIP_IN_PROGRESS.
 * - Transport Marshal Manual Override.
 * - Financial Model: 25% Developer Maintenance Retainer / 75% University Share.
 * - Payment Gateway Subaccount Splits & Cash/Digital Operational Ledger.
 */

class TransitService {
    constructor() {
        this.stations = new Map();
        for (let i = 1; i <= 52; i++) {
            this.stations.set(i, {
                id: i,
                name: `Campus Station #${i}`
            });
        }

        this.currentTripQueue = [];
        this.maxTripCapacity = 3;
        this.activeTrips = [];
        this.vehicleState = 'WAITING_AT_STATION';

        // Financial & Fare Configuration
        this.defaultTicketFare = 150.0; // NGN per trip
        this.developerSharePercent = 25.0; // 25% technical maintenance retainer
        this.universitySharePercent = 75.0; // 75% university revenue

        // Operational Ledger for Cash & Digital
        this.ledger = {
            totalRevenue: 0.0,
            developerRetainerTotal: 0.0,
            universityShareTotal: 0.0,
            transactions: [],
            marshalFloats: new Map() // marshalId -> float balance
        };
    }

    /**
     * @brief Calculates revenue split for a given fare amount.
     */
    calculateRevenueSplit(fareAmount) {
        const developerShare = (fareAmount * this.developerSharePercent) / 100.0;
        const universityShare = (fareAmount * this.universitySharePercent) / 100.0;
        return {
            fareAmount,
            developerShare,
            universityShare
        };
    }

    /**
     * @brief Generates payment gateway split payload (e.g., Paystack/Flutterwave subaccounts).
     */
    generatePaymentSplitPayload(passengerId, originStation, destinationStation, fare = this.defaultTicketFare) {
        const split = this.calculateRevenueSplit(fare);
        return {
            email: `${passengerId}@easy-drive.campus.edu.ng`,
            amount: fare * 100, // in kobo/cents for gateway
            currency: "NGN",
            split: {
                type: "percentage",
                bearer_type: "account",
                subaccounts: [
                    {
                        subaccount: "ACCT_DEVELOPER_MAINTENANCE",
                        share: this.developerSharePercent
                    },
                    {
                        subaccount: "ACCT_UNIVERSITY_TREASURY",
                        share: this.universitySharePercent
                    }
                ]
            },
            metadata: {
                origin: originStation,
                destination: destinationStation
            }
        };
    }

    /**
     * @brief Logs cash or digital transactions into the operational ledger.
     */
    logTransaction(paymentType, passengerId, fare = this.defaultTicketFare, marshalId = null) {
        const split = this.calculateRevenueSplit(fare);
        
        const txn = {
            id: `TXN-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
            paymentType, // 'DIGITAL' or 'CASH_PROXY'
            passengerId,
            fare: split.fareAmount,
            developerRetainer: split.developerShare,
            universityShare: split.universityShare,
            marshalId: marshalId || 'SYSTEM_QR',
            timestamp: Date.now()
        };

        this.ledger.totalRevenue += split.fareAmount;
        this.ledger.developerRetainerTotal += split.developerShare;
        this.ledger.universityShareTotal += split.universityShare;
        this.ledger.transactions.push(txn);

        if (paymentType === 'CASH_PROXY' && marshalId) {
            const currentFloat = this.ledger.marshalFloats.get(marshalId) || 0.0;
            this.ledger.marshalFloats.set(marshalId, currentFloat + split.fareAmount);
        }

        return txn;
    }

    /**
     * @brief Anywhere QR booking from any station (#1 to #52) with dynamic ETA and financial logging.
     */
    bookSeat(passengerId, originStationId, destinationStationId, paymentType = 'DIGITAL', marshalId = null, fare = this.defaultTicketFare) {
        if (!this.stations.has(originStationId) || !this.stations.has(destinationStationId)) {
            return { success: false, message: "Invalid station ID (Must be #1 to #52)." };
        }

        if (this.currentTripQueue.length >= this.maxTripCapacity) {
            return { success: false, message: "Current trip queue is full. Please wait for the next dispatch." };
        }

        const etaMinutes = Math.max(2, Math.abs(originStationId - 3) * 1); 
        const txn = this.logTransaction(paymentType, passengerId, fare, marshalId);

        const booking = {
            passengerId,
            origin: originStationId,
            destination: destinationStationId,
            txnId: txn.id,
            timestamp: Date.now(),
            paid: true
        };

        this.currentTripQueue.push(booking);
        const slotsRemaining = this.maxTripCapacity - this.currentTripQueue.length;

        let dispatchTriggered = false;
        if (this.currentTripQueue.length === this.maxTripCapacity && this.vehicleState === 'WAITING_AT_STATION') {
            dispatchTriggered = this.dispatchTrip();
        }

        return {
            success: true,
            message: `Seat booked successfully. Vehicle arriving in approx ${etaMinutes} mins.`,
            etaMinutes,
            slotsRemaining,
            dispatchTriggered,
            financialSplit: this.calculateRevenueSplit(fare)
        };
    }

    dispatchTrip() {
        if (this.currentTripQueue.length === 0 && this.vehicleState !== 'WAITING_AT_STATION') {
            return false;
        }

        const trip = {
            tripId: `TRIP-${Date.now()}`,
            passengers: [...this.currentTripQueue],
            status: 'TRIP_IN_PROGRESS',
            startTime: Date.now()
        };

        this.activeTrips.push(trip);
        this.currentTripQueue = [];
        this.vehicleState = 'TRIP_IN_PROGRESS';
        return true;
    }

    marshalOverride(action) {
        if (action === 'DISPATCH') {
            return this.dispatchTrip();
        } else if (action === 'RECALL') {
            this.vehicleState = 'WAITING_AT_STATION';
            this.currentTripQueue = [];
            return true;
        }
        return false;
    }

    getFinancialLedgerSummary() {
        return {
            totalRevenue: this.ledger.totalRevenue,
            developerRetainerTotal: this.ledger.developerRetainerTotal,
            universityShareTotal: this.ledger.universityShareTotal,
            totalTransactions: this.ledger.transactions.length,
            marshalFloats: Object.fromEntries(this.ledger.marshalFloats)
        };
    }

    getQueueStatus() {
        return {
            vehicleState: this.vehicleState,
            passengersInQueue: this.currentTripQueue.length,
            slotsRemaining: this.maxTripCapacity - this.currentTripQueue.length,
            activeTripsCount: this.activeTrips.length
        };
    }
}

module.exports = TransitService;
