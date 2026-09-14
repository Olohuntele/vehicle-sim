#include "Vehicle.hpp"
#include <algorithm>

Vehicle::Vehicle() : position(0.0), velocity(0.0), acceleration(0.0) {}

void Vehicle::setControlInput(double input) {
    input = std::clamp(input, -1.0, 1.0);
    if (input >= 0.0) {
        acceleration = input * max_accel;
    } else {
        acceleration = input * max_brake;
    }
}

void Vehicle::update(double dt) {
    velocity += acceleration * dt;
    if (velocity < 0.0) {
        velocity = 0.0;
        acceleration = 0.0;
    }
    position += velocity * dt;
}

double Vehicle::getPosition() const { return position; }
double Vehicle::getVelocity() const { return velocity; }
double Vehicle::getAcceleration() const { return acceleration; }
