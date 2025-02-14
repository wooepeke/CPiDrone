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
    int getBatteryLevel() const;
};

#endif // PIDRONE_H
