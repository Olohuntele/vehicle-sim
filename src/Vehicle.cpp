#include "Vehicle.hpp"
#include <algorithm>

Vehicle::Vehicle() : position(0.0), velocity(0.0), acceleration(0.0), drag_force(0.0) {}

void Vehicle::setControlInput(double input) {
    input = std::clamp(input, -1.0, 1.0);
    if (input >= 0.0) {
        acceleration = input * max_accel;
    } else {
        acceleration = input * max_brake;
    }
}

void Vehicle::update(double dt) {
    drag_force = 0.5 * air_density * cd * frontal_area * velocity * velocity;
    double drag_accel = drag_force / mass;
    double net_accel = acceleration - drag_accel;

    velocity += net_accel * dt;
    if (velocity < 0.0) {
        velocity = 0.0;
    }
    position += velocity * dt;
}

double Vehicle::getPosition() const { return position; }
double Vehicle::getVelocity() const { return velocity; }
double Vehicle::getAcceleration() const { return acceleration; }
double Vehicle::getDragForce() const { return drag_force; }
