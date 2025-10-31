#include <iostream>
#include <thread>
#include <chrono>
#include "lsm9ds0_device.hpp"

int main() {
    try {
        // Configure IMU
        lsm9ds0_device::configure_imu();

        std::cout << "LSM9DS0 configured. Reading sensor values...\n";

        while (true) {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            int16_t mx, my, mz;

            bool aok = lsm9ds0_device::read_accel(ax, ay, az);
            bool gok = lsm9ds0_device::read_gyro(gx, gy, gz);
            bool mok = lsm9ds0_device::read_mag(mx, my, mz);

            if (aok) {
                std::cout << "ACC: " << ax << ", " << ay << ", " << az << "\n";
            } else {
                std::cout << "ACC: read error\n";
            }

            if (gok) {
                std::cout << "GYR: " << gx << ", " << gy << ", " << gz << "\n";
            } else {
                std::cout << "GYR: read error\n";
            }

            if (mok) {
                std::cout << "MAG: " << mx << ", " << my << ", " << mz << "\n";
            } else {
                std::cout << "MAG: read error\n";
            }

            std::cout << "----" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}