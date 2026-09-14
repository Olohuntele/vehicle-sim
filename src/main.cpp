#include <iostream>
#include <iomanip>
#include "Vehicle.hpp"

int main() {
    Vehicle car;
    double dt = 0.1;
    double total_time = 10.0;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=========================================================\n";
    std::cout << " Time (s) | Accel (m/s^2) | Speed (m/s) | Position (m)\n";
    std::cout << "=========================================================\n";

    for (double time = 0.0; time <= total_time + 1e-9; time += dt) {
        if (time < 5.0) {
            car.setControlInput(1.0);
        } else {
            car.setControlInput(-0.8);
        }

        std::cout << std::setw(8) << time << " | "
                  << std::setw(13) << car.getAcceleration() << " | "
                  << std::setw(11) << car.getVelocity() << " | "
                  << std::setw(12) << car.getPosition() << "\n";

        car.update(dt);
    }

    std::cout << "=========================================================\n";
    return 0;
}
