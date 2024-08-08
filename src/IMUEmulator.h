#ifndef IMUEMULATOR_H
#define IMUEMULATOR_H

#include <vector>

class IMUEmulator {
public:
    IMUEmulator(int frequency);
    void processTrajectoryData(const std::string& data);
    std::string generateIMUData();
    void sendIMUData(const std::string& imuData);

private:
    std::vector<std::string> trajectoryData;
    int frequency;
};

#endif

