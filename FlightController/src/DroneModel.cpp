#include "DroneModel.h"
#include <cmath>

// Constructor: Initialize drone with full battery and landed state
DroneModel::DroneModel() : batteryLevel(100), altitude(0.0), isFlying(false), eulerAngles{0.0, 0.0, 0.0}, momentsOfInertia{0.0, 0.0, 0.0} {
    motorSpeeds.fill(0.0); // Initialize all motor speeds to zero
    std::cout << "DroneModel initialized.\n";
}

// Destructor
DroneModel::~DroneModel() {
    stopAllMotors(); // Ensure motors are stopped when shutting down
    std::cout << "DroneModel shutting down.\n";
}

int DroneModel::getBatteryLevel() const {
    return batteryLevel;
}

// Set Euler angles
void DroneModel::setDroneAngles(float roll, float pitch, float yaw) {
    eulerAngles.pitch = pitch;
    eulerAngles.yaw = yaw;
    eulerAngles.roll = roll;
    return;
}

// Get Euler angles
EulerAngles DroneModel::getDroneAngles() const {
    return eulerAngles;
}

// Set MoI
void DroneModel::setMomentsOfInertia() {
    float l_com_x = 0.035; // Length from drone COM to the x position of the motor [m]
    float l_com_y = 0.035; // Length from drone COM to the y position of the motor [m]
    float l_motor = std::sqrt(std::pow(l_com_x, 2) + std::pow(l_com_y, 2)); // Distance from drone COM to motor    
 
    float d_motor = 0.01; // Height of drone COM to motor COM [m]
    float h_motor = 0.01; // Motor height [m]
    float r_motor = 0.01; // Motor radius [m]
    float m_motor = 0.01; // Motor mass [kg]
    
    float l_body = 0.10; // Length of the body of the drone (seen as a rectangle) [m]
    float w_body = 0.05; // Width of the body of the drone (seen as a rectangle) [m]
    float h_body = 0.03; // Height of the body of the drone (seen as a rectangle) [m]
    float D_e = 0.0; // Distance from the COM around the z-axis [m]
    float m_body = 0.1; // Mass of the motor body [kg]

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
MomentsOfInertia DroneModel::getMomentsOfInertia() const {
    return momentsOfInertia;
}

// Initialize PWM pins for the four motors
void DroneModel::initPWMPins(uint pin1, uint pin2, uint pin3, uint pin4) {
    // Store the pin numbers
    pwmPins = {pin1, pin2, pin3, pin4};
    
    // Initialize each pin for PWM output
    for (int i = 0; i < 4; i++) {
        gpio_set_function(pwmPins[i], GPIO_FUNC_PWM);
        
        // Get PWM slice and channel for each pin
        pwmSlices[i] = pwm_gpio_to_slice_num(pwmPins[i]);
        pwmChannels[i] = pwm_gpio_to_channel(pwmPins[i]);
        
        // Configure PWM
        pwm_set_clkdiv(pwmSlices[i], PWM_CLOCK_DIV);
        uint wrap_value = (1 << PWM_RESOLUTION) - 1;  // Max value for 12-bit resolution
        pwm_set_wrap(pwmSlices[i], wrap_value);
        
        // Start with motors stopped
        pwm_set_chan_level(pwmSlices[i], pwmChannels[i], 0);
        
        // Enable PWM
        pwm_set_enabled(pwmSlices[i], true);
    }
    
    std::cout << "PWM initialized for all 4 motors.\n";
}

// Set PWM frequency
void DroneModel::setPWMFrequency(uint frequency) {
    // Calculate and set the clock divider based on the desired frequency
    float clockDiv = (float)clock_get_hz(clk_sys) / (frequency * (1 << PWM_RESOLUTION));
    
    for (int i = 0; i < 4; i++) {
        pwm_set_clkdiv(pwmSlices[i], clockDiv);
    }
    
    std::cout << "PWM frequency set to " << frequency << " Hz.\n";
}

// Set speed for a specific motor (0.0 to 1.0)
void DroneModel::setMotorSpeed(uint motorIndex, float speed) {
    if (motorIndex >= 4) {
        std::cout << "Invalid motor index! Must be 0-3.\n";
        return;
    }
    
    // Clamp speed between 0.0 and 1.0
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;
    
    motorSpeeds[motorIndex] = speed;
    
    // Calculate PWM level based on resolution
    uint level = (uint)(speed * ((1 << PWM_RESOLUTION) - 1));
    
    // Set PWM level
    pwm_set_chan_level(pwmSlices[motorIndex], pwmChannels[motorIndex], level);
}

// Set speed for all motors at once
void DroneModel::setAllMotorSpeeds(float speed1, float speed2, float speed3, float speed4) {
    setMotorSpeed(0, speed1);
    setMotorSpeed(1, speed2);
    setMotorSpeed(2, speed3);
    setMotorSpeed(3, speed4);
}

// Get current speed for a specific motor
float DroneModel::getMotorSpeed(uint motorIndex) const {
    if (motorIndex >= 4) {
        std::cout << "Invalid motor index! Must be 0-3.\n";
        return 0.0;
    }
    
    return motorSpeeds[motorIndex];
}

// Stop all motors
void DroneModel::stopAllMotors() {
    setAllMotorSpeeds(0.0, 0.0, 0.0, 0.0);
    std::cout << "All motors stopped.\n";
}