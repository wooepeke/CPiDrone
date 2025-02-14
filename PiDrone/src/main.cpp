#include <iostream>
#include "PiDrone.h"

int main() {
    PiDrone drone;

    drone.takeoff();
    drone.adjustAltitude(10.5);
    std::cout << "Battery: " << drone.checkBattery() << "%\n";
    drone.land();

    return 0;
}


