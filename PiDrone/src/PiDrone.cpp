#include "PiDrone.h"

// Constructor: Initialize drone with full battery and landed state
PiDrone::PiDrone() : batteryLevel(100), altitude(0.0), isFlying(false) {
    std::cout << "PiDrone initialized.\n";
}

// Destructor
PiDrone::~PiDrone() {
    std::cout << "PiDrone shutting down.\n";
}

int PiDrone::getBatteryLevel() const {
    return batteryLevel;
}