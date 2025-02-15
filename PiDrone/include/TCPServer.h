#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <iostream>
#include <string>
#include <unistd.h>

#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>

#include "PiDrone.h"  // Include the PiDrone header

using json = nlohmann::json;

class TCPServer {
private:
    int server_fd, client_socket;
    struct sockaddr_in address;
    int port;
    bool isRunning;
    PiDrone &drone;  // Reference to the PiDrone instance

public:
    TCPServer(int port, PiDrone &drone);
    ~TCPServer();

    // Methods
    static std::string getLocalIP();
    void start();
    void stop();
    json getSensorData();
    void sendData();
};

#endif  // TCPSERVER_H
