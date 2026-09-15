#ifndef VEHICLE_HPP
#define VEHICLE_HPP

class Vehicle {
private:
    double position;
    double velocity;
    double acceleration;
    double drag_force;
    double brake_force;
    double weight_transfer;
    const double max_accel = 3.5;
    const double max_brake = 6.0;
    const double mass = 1200.0;         // kg
    const double air_density = 1.225;   // kg/m^3
    const double cd = 0.3;              // drag coefficient
    const double frontal_area = 2.2;    // m^2
    const double cg_height = 0.5;       // m (Center of gravity height)
    const double wheelbase = 2.5;       // m
    const double g = 9.81;              // m/s^2
    const double front_weight_ratio = 0.6; // 60% static front axle weight ratio

public:
    Vehicle(double initial_velocity = 0.0);
    void setVelocity(double v);
    void setControlInput(double input);
    void setBrakeInput(double brake);
    void update(double dt);
    double getPosition() const;
    double getVelocity() const;
    double getAcceleration() const;
    double getDragForce() const;
    double getBrakeForce() const;
    double getWeightTransfer() const;
    double getFrontAxleLoad() const;
    double getMaxBrake() const;
};

#endif
