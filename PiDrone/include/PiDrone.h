#ifndef PIDRONE_H
#define PIDRONE_H

#include <iostream>

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

struct MomentsOfInertia {
    float Ixx; 
    float Iyy; 
    float Izz;
};

// Overload the << operator for EulerAngles
// Defines how to print the angles variable
inline std::ostream& operator<<(std::ostream& os, const EulerAngles& angles) {
    os << "Roll: " << angles.roll << ", Pitch: " << angles.pitch << ", Yaw: " << angles.yaw;
    return os;
}

class PiDrone {
private:
    int batteryLevel;                       // Battery percentage (0-100)
    float altitude;                         // Altitude in meters
    bool isFlying;                          // Drone flight status
    EulerAngles eulerAngles;                // Drone euler angles
    MomentsOfInertia momentsOfInertia;      // Drone MoI

public: 
    // Constructor
    PiDrone();

    // Destructor
    ~PiDrone();

    // Methods
    int getBatteryLevel() const;

    void setDroneAngles(float roll, float pitch, float yaw);
    EulerAngles getDroneAngles() const;

    void setMomentsOfInertia();
    MomentsOfInertia getMomentsOfInertia() const;
};

#endif // PIDRONE_H
