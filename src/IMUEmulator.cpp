#include "IMUEmulator.h"
#include "IMUInterface.h"
#include <iostream>

IMUEmulator::IMUEmulator(int frequency) : frequency(frequency) {}

void IMUEmulator::processTrajectoryData(const std::string& data) {
    trajectoryData.push_back(data);
    if (trajectoryData.size() > 2) {
        trajectoryData.erase(trajectoryData.begin());  // Keep only the latest two points
    }
}

std::string IMUEmulator::generateIMUData() {
    if (trajectoryData.size() < 2) {
        return "";
    }
    // Placeholder for transformation algorithm
    return "IMU data based on " + trajectoryData[0] + " and " + trajectoryData[1];
}

void IMUEmulator::sendIMUData(const std::string& imuData) {
    IMUInterface imuInterface;
    imuInterface.sendToBoard(imuData);
}

