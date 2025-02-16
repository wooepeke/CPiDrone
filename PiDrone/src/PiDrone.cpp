#include "PiDrone.h"

// Constructor: Initialize drone with full battery and landed state
PiDrone::PiDrone() : batteryLevel(100), altitude(0.0), isFlying(false), eulerAngles{0.0, 0.0, 0.0}, momentsOfInertia{0.0, 0.0, 0.0} {
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

// Get Euler angles
EulerAngles PiDrone::getDroneAngles() const {
    return eulerAngles;
}

// Set MoI
void PiDrone::setMomentsOfInertia() {
    float l_com_x = 0.035                                                                     // Length from drone COM to the x position of the motor [m]
    float l_com_y = 0.035                                                                     // Length from drone COM to the y position of the motor [m]
    float l_motor = np.sqrt(l_com_x**2 + l_com_y**2)                                          // Length from drone COM to the motor [m]
    float d_motor = 0                                                                         // Height of drone COM to motor COM [m]
    float h_motor = 0                                                                         // Motor height [m]
    float r_motor = 0                                                                         // Motor radius [m]
    float m_motor = 0                                                                         // Motor mass [kg]

    // Moments of inertia of one motor
    float Ixx_motor = m_motor * ((r_motor**2)/4 + (h_motor**2)/12 + d_motor**2)               // Moment of inertia of a motor around the x-axis
    float Iyy_motor = m_motor * ((r_motor**2)/4 + (h_motor**2)/12 + l_motor**2 + d_motor**2)  // Moment of inertia of a motor around the y-axis
    float Izz_motor = m_motor * ((r_motor**2)/2 + l_motor**2)                                 // Moment of inertia of a motor around the z-axis

    // Moments of inertia of the drone
    float Ixx = 0                                                                             // Moment of inertia around the x-axis
    float Iyy = 0                                                                             // Moment of inertia around the y-axis
    float Izz = 0                                                                             // Moment of inertia around the z-axis    
    return;
}

// Get MoI
MomentsOfInertia PiDrone::getMomentsOfInertia() const {
    return momentsOfInertia;
}