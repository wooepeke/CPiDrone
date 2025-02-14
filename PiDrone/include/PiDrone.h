#ifndef PIDRONE_H
#define PIDRONE_H

#include <iostream>

class PiDrone {
private:
    int batteryLevel; // Battery percentage (0-100)
    float altitude;   // Altitude in meters
    bool isFlying;    // Drone flight status

public:
    // Constructor
    PiDrone();

    // Destructor
    ~PiDrone();

    // Methods
    void takeoff();
    void land();
    void adjustAltitude(float newAltitude);
    int checkBattery() const;
    
    // Getters
    float getAltitude() const;
    bool getStatus() const;
};

#endif // PIDRONE_H
