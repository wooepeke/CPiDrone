#include "PiDrone.h"

// Constructor: Initialize drone with full battery and landed state
PiDrone::PiDrone() : batteryLevel(100), altitude(0.0), isFlying(false), eulerAngles{0.0, 0.0, 0.0} {
    std::cout << "PiDrone initialized.\n";
}

// Destructor
PiDrone::~PiDrone() {   
    std::cout << "PiDrone shutting down.\n";
}

int PiDrone::getBatteryLevel() const {
    return batteryLevel;
}

// Set Euler angles
void PiDrone::setDroneAngles(float roll, float pitch, float yaw) {
    eulerAngles.pitch = pitch;
    eulerAngles.yaw = yaw;
    eulerAngles.roll = roll;
    return;
}

EulerAngles PiDrone::getDroneAngles() const {
    return eulerAngles;
}