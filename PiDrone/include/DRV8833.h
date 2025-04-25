#ifndef DRV8833_H
#define DRV8833_H

#include <pigpio.h>
#include <chrono>  // For time-related functionality

class DRV8833 {
public:
    // Constructor & Destructor
    DRV8833(int pin1, int pin2, int minInput, int maxInput, int neutralWidth, bool invert = false, bool doublePWM = false);
    ~DRV8833();
    
    // Drive function
    void drive(int controlValue, int maxPWM, int rampTime, bool brake = false, bool neutralBrake = false);

private:
    int _pin1, _pin2;               // Motor control pins
    int _minInput, _maxInput;       // Min & Max input value
    int _minNeutral, _maxNeutral;   // Neutral range values
    int _controlValueRamp;          // Ramped control value
    bool _invert;                   // Direction invert flag
    bool _doublePWM;                // Double PWM mode flag
    bool _brake;                    // Brake control flag
    bool _neutralBrake;             // Neutral brake flag
    int _controlValue;              // Control value input
    int _maxPWM;                    // Maximum PWM value
    int _rampTime;                  // Ramp time (milliseconds)
    int _state;                     // Motor state
    std::chrono::steady_clock::time_point _previousMillis;  // Time point for ramp control
};

#endif // DRV8833_H
