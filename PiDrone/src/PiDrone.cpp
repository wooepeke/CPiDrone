#include "PiDrone.h"
#include <cmath>

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
    float l_com_x = 0.035; // Length from drone COM to the x position of the motor [m]
    float l_com_y = 0.035; // Length from drone COM to the y position of the motor [m]
    float l_motor = std::sqrt(std::pow(l_com_x, 2) + std::pow(l_com_y, 2)); // Distance from drone COM to motor    
 
    float d_motor = 0.0; // Height of drone COM to motor COM [m]
    float h_motor = 0.0; // Motor height [m]
    float r_motor = 0.0; // Motor radius [m]
    float m_motor = 0.0; // Motor mass [kg]
     
    float l_body = 0.0; // Length of the body of the drone (seen as a rectangle) [m]
    float w_body = 0.0; // Width of the body of the drone (seen as a rectangle) [m]
    float h_body = 0.0; // Height of the body of the drone (seen as a rectangle) [m]
    float D_e = 0.0; // Distance from the COM around the z-axis [m]
    float m_body = 0.0; // Mass of the motor body [kg]

    // Moments of inertia of one motor
    float Ixx_motor = m_motor * ((std::pow(r_motor, 2) / 4) + (std::pow(h_motor, 2) / 12) + std::pow(d_motor, 2));
    float Iyy_motor = m_motor * ((std::pow(r_motor, 2) / 4) + (std::pow(h_motor, 2) / 12) + std::pow(l_motor, 2) + std::pow(d_motor, 2));
    float Izz_motor = m_motor * ((std::pow(r_motor, 2) / 2) + std::pow(l_motor, 2));

    // Moments of inertia of the body
    float Ixx_body = m_body * ((std::pow(w_body, 2) / 12) + (std::pow(h_body, 2) / 12) + std::pow(D_e, 2));
    float Iyy_body = m_body * ((std::pow(l_body, 2) / 12) + (std::pow(h_body, 2) / 12) + std::pow(D_e, 2));
    float Izz_body = m_body * ((std::pow(l_body, 2) / 12) + (std::pow(w_body, 2) / 12));

    // Moments of inertia of the drone
    float Ixx = Ixx_motor + Ixx_body; // Moment of inertia around the x-axis
    float Iyy = Iyy_motor + Iyy_body; // Moment of inertia around the y-axis
    float Izz = Izz_motor + Izz_body; // Moment of inertia around the z-axis

    momentsOfInertia.Ixx = Ixx;
    momentsOfInertia.Iyy = Iyy;
    momentsOfInertia.Izz = Izz;
    return;
}

// Get MoI
MomentsOfInertia PiDrone::getMomentsOfInertia() const {
    return momentsOfInertia;
}