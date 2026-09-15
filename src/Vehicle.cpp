#include "Vehicle.hpp"
#include <algorithm>

Vehicle::Vehicle() 
    : position(0.0), velocity(0.0), acceleration(0.0), drag_force(0.0), brake_force(0.0), weight_transfer(0.0) {}

void Vehicle::setControlInput(double input) {
    input = std::clamp(input, -1.0, 1.0);
    if (input >= 0.0) {
        acceleration = input * max_accel;
        brake_force = 0.0;
    } else {
        acceleration = 0.0;
        setBrakeInput(-input);
    }
}

void Vehicle::setBrakeInput(double brake) {
    brake = std::clamp(brake, 0.0, 1.0);
    brake_force = brake * mass * max_brake;
}

void Vehicle::update(double dt) {
    drag_force = 0.5 * air_density * cd * frontal_area * velocity * velocity;
    
    // Apply brake force only if vehicle is moving
    double effective_brake_force = (velocity > 0.0) ? brake_force : 0.0;
    
    double drive_force = acceleration * mass;
    double net_force = drive_force - drag_force - effective_brake_force;
    
    double net_accel = net_force / mass;

    velocity += net_accel * dt;
    if (velocity < 0.0) {
        velocity = 0.0;
        net_accel = 0.0;
    }
    position += velocity * dt;

    // Longitudinal weight transfer onto front axle: Delta W = (m * (-a_x) * h) / L
    weight_transfer = (mass * (-net_accel) * cg_height) / wheelbase;
}

double Vehicle::getPosition() const { return position; }
double Vehicle::getVelocity() const { return velocity; }
double Vehicle::getAcceleration() const { return acceleration; }
double Vehicle::getDragForce() const { return drag_force; }
double Vehicle::getBrakeForce() const { return brake_force; }
double Vehicle::getWeightTransfer() const { return weight_transfer; }
double Vehicle::getFrontAxleLoad() const {
    double static_front_load = mass * g * front_weight_ratio;
    return static_front_load + weight_transfer;
}
