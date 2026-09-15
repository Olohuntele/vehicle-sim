#ifndef VEHICLE_HPP
#define VEHICLE_HPP

class Vehicle {
private:
    double position;
    double velocity;
    double acceleration;
    double drag_force;
    const double max_accel = 3.5;
    const double max_brake = 6.0;
    const double mass = 1200.0;         // kg
    const double air_density = 1.225;   // kg/m^3
    const double cd = 0.3;              // drag coefficient
    const double frontal_area = 2.2;    // m^2

public:
    Vehicle();
    void setControlInput(double input);
    void update(double dt);
    double getPosition() const;
    double getVelocity() const;
    double getAcceleration() const;
    double getDragForce() const;
};

#endif
