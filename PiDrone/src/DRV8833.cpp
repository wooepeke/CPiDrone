/*
DRV8833.h - Library for the Texas Instruments DRV8833 motor driver.
Created by TheDIYGuy999 June 2016
Released into the public domain.
*/

#include <iostream>
#include <pigpio.h>
#include "DRV8833.h"
#include <chrono>  // For time-related functionality


// Member definition (code) ========================================================================
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// NOTE: The first pin must always be PWM capable, the second only, if the last parameter is set to "true"
// SYNTAX: IN1, IN2, min. input value, max. input value, neutral position width
// invert rotation direction, true = both pins are PWM capable
DRV8833::DRV8833(int pin1, int pin2, int minInput, int maxInput, int neutralWidth, bool invert, bool doublePWM) { 
    _pin1 = pin1;
    _pin2 = pin2;
    _minInput = minInput;
    _maxInput = maxInput;
    _minNeutral = (_maxInput + _minInput) / 2 - (neutralWidth / 2);
    _maxNeutral = (_maxInput + _minInput) / 2 + (neutralWidth / 2);
    _controlValueRamp = (_minNeutral + _maxNeutral) / 2;
    _invert = invert;
    _doublePWM = doublePWM;
    _state = 0;

    // Initialize _previousMillis to current time
    _previousMillis = std::chrono::steady_clock::now();

    gpioSetMode(_pin1, PI_OUTPUT);
    gpioSetMode(_pin2, PI_OUTPUT);
    gpioWrite(_pin1, 0);
    gpioWrite(_pin2, 0);
}

DRV8833::~DRV8833() {
    gpioWrite(_pin1, 0);
    gpioWrite(_pin2, 0);
}

// Drive function ************************************************************

// SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
// true = brake active, false = brake in neutral position inactive
void DRV8833::drive(int controlValue, int maxPWM, int rampTime, bool brake, bool neutralBrake) {
    _controlValue = controlValue;
    _maxPWM = maxPWM;
    _rampTime = rampTime;
    _brake = brake;
    _neutralBrake = neutralBrake;

    if (_invert) {
        _controlValue = _maxInput + _minInput - _controlValue; // invert driving direction
    }

    // Fader (allows to ramp the motor speed slowly up & down) --------------------
    if (_rampTime >= 1) {
        auto currentMillis = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - _previousMillis).count() >= _rampTime) {
            if (_controlValue > _controlValueRamp && _controlValueRamp < _maxInput) {
                _controlValueRamp++;
            } else if (_controlValue < _controlValueRamp && _controlValueRamp > _minInput) {
                _controlValueRamp--;
            }
            _previousMillis = currentMillis;
        }
    } else {
        _controlValueRamp = _controlValue;
    }
    
    // H bridge controller -------------------
    if (_doublePWM) { // Mode with two PWM capable pins (both pins must be PWM capable!)
        if (!_brake) { // Coast mode (fast decay)
            if (_controlValueRamp >= _maxNeutral) { // Forward
                gpioWrite(_pin1, 0);
                gpioPWM(_pin2, (_controlValueRamp - _maxNeutral) * _maxPWM / (_maxInput - _maxNeutral));
            } else if (_controlValueRamp <= _minNeutral) { // Reverse
                gpioWrite(_pin2, 0);
                gpioPWM(_pin1, (_minNeutral - _controlValueRamp) * _maxPWM / (_minNeutral - _minInput));
            } else { // Neutral
                gpioWrite(_pin1, 0);
                gpioWrite(_pin2, 0);
            }
        }
        else { // Brake mode (slow decay)
            if (_controlValueRamp >= _maxNeutral) { // Forward
                gpioWrite(_pin2, 1);
                gpioPWM(_pin1, 255 - (_controlValueRamp - _maxNeutral) * _maxPWM / (_maxInput - _maxNeutral));
            } 
            else if (_controlValueRamp <= _minNeutral) { // Reverse
                gpioWrite(_pin1, 1);
                gpioPWM(_pin2, 255 - (_minNeutral - _controlValueRamp) * _maxPWM / (_minNeutral - _minInput));
            } 
            else { // Neutral
                gpioWrite(_pin1, _neutralBrake ? 1 : 0);
                gpioWrite(_pin2, _neutralBrake ? 1 : 0);
            }
        }
    }
    else { // Mode with only one PWM capable pin (pin 1 = PWM, pin2 = direction) -----
        // NOTE: the brake is always active in one direction and always inactive in the other!
        // Only use this mode, if your microcontroller does not have enough PWM capable pins!
        // If the brake is active in the wrong direction, simply switch both motor wires and
        // change the "invert" boolean!
        if (_controlValueRamp >= _maxNeutral) { // Forward
            gpioWrite(_pin2, 1);
            gpioPWM(_pin1, 255 - map(_controlValueRamp, _maxNeutral, _maxInput, 0, _maxPWM));
            std::cout << "Forward speed: " << 255 - map(_controlValueRamp, _maxNeutral, _maxInput, 0, _maxPWM) << std::endl;
        } 
        else if (_controlValueRamp <= _minNeutral) { // Reverse
            gpioWrite(_pin2, 0);
            gpioPWM(_pin1, (map(_controlValueRamp, _minNeutral, _minInput, 0, _maxPWM)));
            std::cout << "Backward speed: " << (map(_controlValueRamp, _minNeutral, _minInput, 0, _maxPWM)) << std::endl;
        } 
        else { // Neutral
            gpioWrite(_pin1, 1);
            gpioWrite(_pin2, 1);
        }
    }
}
