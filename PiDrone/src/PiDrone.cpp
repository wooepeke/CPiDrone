#include "PiDrone.h"

// Constructor: Initialize drone with full battery and landed state
PiDrone::PiDrone() : batteryLevel(100), altitude(0.0), isFlying(false) {
    std::cout << "PiDrone initialized.\n";
}

// Destructor
PiDrone::~PiDrone() {
    std::cout << "PiDrone shutting down.\n";
}

// Takeoff function
void PiDrone::takeoff() {
    if (batteryLevel > 10 && !isFlying) {
        isFlying = true;
        altitude = 1.0; // Default takeoff altitude
        std::cout << "PiDrone is taking off.\n";
    } else {
        std::cout << "Takeoff failed: Low battery or already flying.\n";
    }
}

// Land function
void PiDrone::land() {
    if (isFlying) {
        altitude = 0.0;
        isFlying = false;
        std::cout << "PiDrone has landed.\n";
    } else {
        std::cout << "PiDrone is already on the ground.\n";
    }
}

// Adjust altitude
void PiDrone::adjustAltitude(float newAltitude) {
    if (isFlying && newAltitude >= 0.0) {
        altitude = newAltitude;
        std::cout << "Altitude adjusted to " << altitude << " meters.\n";
    } else {
        std::cout << "Cannot adjust altitude: Either not flying or invalid altitude.\n";
    }
}

// Check battery level
int PiDrone::checkBattery() const {
    return batteryLevel;
}

// Get altitude
float PiDrone::getAltitude() const {
    return altitude;
}

// Get flight status
bool PiDrone::getStatus() const {
    return isFlying;
}
