#ifndef DRONE_MODEL_H
#define DRONE_MODEL_H

#include <iostream>
#include <array>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Structure for Euler angles
struct EulerAngles {
    float roll;    // Roll angle in radians
    float pitch;   // Pitch angle in radians
    float yaw;     // Yaw angle in radians
};

// Structure for moments of inertia
struct MomentsOfInertia {
    float Ixx;     // Moment of inertia around x-axis
    float Iyy;     // Moment of inertia around y-axis
    float Izz;     // Moment of inertia around z-axis
};

class DroneModel {
private:
    int batteryLevel;           // Battery level percentage
    float altitude;             // Current altitude in meters
    bool isFlying;              // Flying status
    EulerAngles eulerAngles;    // Drone orientation
    MomentsOfInertia momentsOfInertia; // Moments of inertia
    
    // PWM control for motors
    std::array<uint, 4> pwmPins;       // GPIO pins for PWM output
    std::array<uint, 4> pwmSlices;     // PWM slice numbers
    std::array<uint, 4> pwmChannels;   // PWM channels
    std::array<float, 4> motorSpeeds;  // Motor speeds (0.0 to 1.0)
    
    const uint PWM_FREQ = 25000;       // PWM frequency in Hz
    const uint PWM_RESOLUTION = 12;    // PWM resolution (12 bits = 0-4095)
    const uint PWM_CLOCK_DIV = 1;      // PWM clock divider
    
public:
    // Constructor and destructor
    DroneModel();
    ~DroneModel();
    
    // Battery management
    int getBatteryLevel() const;
    
    // Orientation control
    void setDroneAngles(float roll, float pitch, float yaw);
    EulerAngles getDroneAngles() const;
    
    // Physics properties
    void setMomentsOfInertia();
    MomentsOfInertia getMomentsOfInertia() const;
    
    // PWM motor control
    void initPWMPins(uint pin1, uint pin2, uint pin3, uint pin4);
    void setPWMFrequency(uint frequency);
    void setMotorSpeed(uint motorIndex, float speed); // 0.0 to 1.0
    void setAllMotorSpeeds(float speed1, float speed2, float speed3, float speed4);
    float getMotorSpeed(uint motorIndex) const;
    void stopAllMotors();
};

#endif // DRONE_MODEL_H