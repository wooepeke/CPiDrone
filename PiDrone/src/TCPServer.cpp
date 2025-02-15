#include "TCPServer.h"
#include <cstring>
#include <cstdlib>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cmath>
#include <iomanip>
#include <sstream>

// Constructor: Now accepts a reference to a PiDrone object
TCPServer::TCPServer(int port, PiDrone &drone) : port(port), isRunning(false), drone(drone) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
}

// Destructor
TCPServer::~TCPServer() {
    stop();
    close(server_fd);
}

// Start the TCP server
void TCPServer::start() {
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << port << "...\n";
    isRunning = true;

    while (isRunning) {
        int addrlen = sizeof(address);
        client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        if (client_socket < 0) {
            perror("Accept failed");
            continue;
        }

        std::cout << "Client connected!\n";
        while (isRunning) {
            sendData();
            usleep(100000);
        }

        close(client_socket);
    }
}

// Stop the TCP server
void TCPServer::stop() {
    isRunning = false;
    close(client_socket);
}

// Get the pi IP address
std::string TCPServer::getLocalIP() {
    struct ifaddrs *ifaddr, *ifa;
    char ip[INET_ADDRSTRLEN] = {0};

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return "Error";
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) { // IPv4
            struct sockaddr_in *sa = (struct sockaddr_in *)ifa->ifa_addr;
            inet_ntop(AF_INET, &(sa->sin_addr), ip, INET_ADDRSTRLEN);

            // Ignore localhost (127.0.0.1)
            if (std::string(ip) != "127.0.0.1") {
                freeifaddrs(ifaddr);
                return std::string(ip);
            }
        }
    }

    freeifaddrs(ifaddr);
    return "No valid IP found";
}


json TCPServer::getSensorData() {
    json data;

    EulerAngles angles = drone.getDroneAngles();  // Fetch current drone angles

    // Round Euler angles to two decimal places using a more reliable approach
    angles.roll = round(angles.roll * 100.0) / 100.0;
    angles.pitch = round(angles.pitch * 100.0) / 100.0;
    angles.yaw = round(angles.yaw * 100.0) / 100.0;

    // Convert the rounded angles to strings with 2 decimal places
    std::ostringstream roll_stream, pitch_stream, yaw_stream;
    roll_stream << std::fixed << std::setprecision(2) << angles.roll;
    pitch_stream << std::fixed << std::setprecision(2) << angles.pitch;
    yaw_stream << std::fixed << std::setprecision(2) << angles.yaw;

    // Add the rounded angles as strings to the JSON object
    data["euler_angles"] = {roll_stream.str(), pitch_stream.str(), yaw_stream.str()};

    return data; // Return the JSON object directly
}

// Send sensor data to the client
void TCPServer::sendData() {
    json data = getSensorData();  // Get JSON object data

    // Serialize JSON object to string
    std::string jsonData = data.dump(); // Convert to string

    // Add a newline after the JSON data
    jsonData += "\n";

    // Send the data over the socket
    send(client_socket, jsonData.c_str(), jsonData.size(), 0);
    std::cout << "Sent: " << jsonData << std::endl;
}


