#include "lib/main.hpp"
#include <iostream>

int main() {
UKF ukf;
    x << 1,
            0,
            0,
            0,
            0,
            0,
            0;

        ukf.initialize(x);

    for (int i = 0; i < 1; ++i) {
        Eigen::Vector3d gyro(0, 0, .01);
        Eigen::Vector3d accel(0, 0, -9.81);
        // Update and predict new states using measurement data and release update states
            x = ukf.update(x, .001, accel, gyro);
            qw = x(0);
            qx = x(1);
            qy = x(2);
            qz = x(3);
            gyrox = x(4);
            gyroy = x(5);
            gyroz = x(6);

        std::cout << "q = [" << qw << ", " << qx << ", " << qy << ", " << qz <<", " << gyrox <<", " << gyroy <<", " << gyroz << "]\n";
    }
}
