#include <iostream>
#include <iomanip>
#include <algorithm>
#include "Vehicle.hpp"

int main() {
    double initial_speed = 16.67; // 60 km/h in m/s
    double obstacle_distance = 10.0; // initial distance to obstacle in meters
    double dt = 0.1;
    double max_sim_time = 10.0;

    Vehicle car(initial_speed);
    double obstacle_position = obstacle_distance;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "===================================================================================================\n";
    std::cout << " Time (s) | Speed (m/s) | Remaining Dist (m) | Req Decel (m/s^2) | Applied Brake Force (N)\n";
    std::cout << "===================================================================================================\n";

    for (double time = 0.0; time <= max_sim_time; time += dt) {
        double current_speed = car.getVelocity();
        double remaining_dist = obstacle_position - car.getPosition();

        double req_decel = 0.0;
        if (remaining_dist > 0.0 && current_speed > 0.0) {
            req_decel = (current_speed * current_speed) / (2.0 * remaining_dist);
        } else if (remaining_dist <= 0.0 && current_speed > 0.0) {
            req_decel = car.getMaxBrake();
        }

        double brake_input = req_decel / car.getMaxBrake();
        car.setControlInput(-brake_input);

        std::cout << std::setw(8) << time << " | "
                  << std::setw(11) << current_speed << " | "
                  << std::setw(18) << std::max(0.0, remaining_dist) << " | "
                  << std::setw(17) << req_decel << " | "
                  << std::setw(22) << car.getBrakeForce() << "\n";

        if (current_speed <= 0.0 && time > 0.0) {
            std::cout << "===================================================================================================\n";
            std::cout << "Vehicle stopped successfully.\n";
            break;
        }

        car.update(dt);
    }

    return 0;
}
