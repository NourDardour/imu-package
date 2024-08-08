#include "IMUInterface.h"
#include <iostream>

void IMUInterface::sendToBoard(const std::string& imuData) {
    // Implement communication protocol
    std::cout << "Sending IMU data to the board: " << imuData << std::endl;
}

