#include "IMUEmulator.h"
#include <cassert>

void testProcessTrajectoryData() {
    IMUEmulator emulator(1000);
    emulator.processTrajectoryData("data1");
    emulator.processTrajectoryData("data2");
    std::string imuData = emulator.generateIMUData();
    assert(!imuData.empty());
}

int main() {
    testProcessTrajectoryData();
    // Add more tests as needed
    return 0;
}

