#ifndef VEHICLE_HPP
#define VEHICLE_HPP

class Vehicle {
private:
    double position;
    double velocity;
    double acceleration;
    const double max_accel = 3.5;
    const double max_brake = 6.0;

public:
    Vehicle();
    void setControlInput(double input);
    void update(double dt);
    double getPosition() const;
    double getVelocity() const;
    double getAcceleration() const;
};

#endif
