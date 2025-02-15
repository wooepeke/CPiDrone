#include <iostream>
#include <chrono>  // Include for timing
#include <unistd.h>  // For usleep
#include <thread>

#include "PiDrone.h"
#include "MPU6050.h"
#include "TCPServer.h"

MPU6050 device(0x68);

int main() {
    PiDrone drone;
    float ax, ay, az, gr, gp, gy; // Variables to store the accel, gyro, and angle values

	sleep(1); //Wait for the MPU6050 to stabilize

    // Calculate the offsets
	// std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	// device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	// std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";

    // Read the current yaw angle
    device.calc_yaw = false;

    TCPServer server(8080, drone);

    std::string ip = server.getLocalIP();
    std::cout << "Raspberry Pi Local IP: " << ip << std::endl;

    std::thread serverThread([&server]() {
        server.start();
    });
    
    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // // Get the current accelerometer values
        // device.getAccel(&ax, &ay, &az);
        // std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

        // // Get the current gyroscope values
        // device.getGyro(&gr, &gp, &gy);
        // std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";

		device.getAngle(0, &gr);
		device.getAngle(1, &gp);
		device.getAngle(2, &gy);

        drone.setDroneAngles(gr, gp, gy);
        
		// std::cout << "Current angle around the roll axis: " << gr << "\n";
		// std::cout << "Current angle around the pitch axis: " << gp << "\n";
		// std::cout << "Current angle around the yaw axis: " << gy << "\n";

        usleep(1000); //0.25sec

        // End time measurement
        auto end_time = std::chrono::high_resolution_clock::now();
        // usleep(500); // 5 milli seconds delay
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

        // Print loop duration in milliseconds
        // std::cout << "Loop duration: " << elapsed.count() << " ms\n";

    }

    return 0;
}
