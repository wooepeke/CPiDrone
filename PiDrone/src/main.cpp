#include <iostream>
#include <chrono>  // Include for timing
#include <unistd.h>  // For usleep
#include <thread>
#include <pigpio.h>
#include <csignal>
#include <atomic>  // Add this header for std::atomic

#include "PiDrone.h"
#include "MPU6050.h"
#include "TCPServer.h"
#include "DRV8833.h"
#include <PCA9685.h>


MPU6050 device(0x68);
PiPCA9685::PCA9685 pca{};
std::atomic<bool> running(true);  // Shared flag for stopping

// Run this for mpu6050:
// sudo killall pigpiod

// Shutdown drone
void shutdown(int signum) {
    std::cout << "\n[INFO] Received signal " << signum << " -> Shutting down...\n";
    
    pca.set_all_pwm(0, 0);  // Turn off all LEDs/PWM

    std::cout << "[INFO] All PWM channels turned off. Exiting now.\n";
    running = false;
    exit(signum);
}

// Listen to q
void keyListener() {
    char ch;
    while (running) {
        std::cin >> ch;
        if (ch == 'q') {
            std::cout << "\n[INFO] 'q' key pressed -> Exiting program...\n";
            shutdown(SIGTERM);
        }
    }
}

int main() {
    // Handle interrupts like CTRL+C
    signal(SIGINT, shutdown);
    signal(SIGTERM, shutdown);

    // Start a separate thread for keyboard input
    std::thread inputThread(keyListener);

    // initialize drone
    PiDrone drone;

    float ax, ay, az, gr, gp, gy; // Variables to store the accel, gyro, and angle values

	sleep(1); //Wait for the MPU6050 to stabilize

    pca.set_pwm_freq(1048.0);
  
    int channel = 1;  // Change this to the correct LED channel  

    // Calculate the offsets
	// std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	// device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	// std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";

    // Read the current yaw angle
    device.calc_yaw = false;
    
    // TCPServer server(8080, drone);

    // std::string ip = server.getLocalIP();
    // std::cout << "Raspberry Pi Local IP: " << ip << std::endl;

    // std::thread serverThread([&server]() {
    //     server.start();
    // });
    
    // while (true) {
    //     auto start_time = std::chrono::high_resolution_clock::now();

    //     // // Get the current accelerometer values
    //     // device.getAccel(&ax, &ay, &az);
    //     // std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

    //     // // Get the current gyroscope values
    //     // device.getGyro(&gr, &gp, &gy);
    //     // std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";

	// 	device.getAngle(0, &gr);
	// 	device.getAngle(1, &gp);
	// 	device.getAngle(2, &gy);

    //     drone.setDroneAngles(-gr, gp, gy);
        
	// 	// std::cout << "Current angle around the roll axis: " << gr << "\n";
	// 	// std::cout << "Current angle around the pitch axis: " << gp << "\n";
	// 	// std::cout << "Current angle around the yaw axis: " << gy << "\n";

    //     usleep(1000); //0.25sec

    //     // End time measurement
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     // usleep(500); // 5 milli seconds delay
    //     std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

    //     // Print loop duration in milliseconds
    //     // std::cout << "Loop duration: " << elapsed.count() << " ms\n";

    // }

    while (true) {
        // Fade in
        for (int dutyCycle = 0; dutyCycle <= 4095; dutyCycle += 10) {
            pca.set_pwm(channel, 0, dutyCycle);
            usleep(5000);  // Small delay for smooth fading (5ms)
        }
  
        // Fade out
        for (int dutyCycle = 4095; dutyCycle >= 0; dutyCycle -= 10) {
          pca.set_pwm(channel, 0, dutyCycle);
            usleep(5000);
        }
    }

    inputThread.join();  // Wait for the input thread to finish
    return 0;
}