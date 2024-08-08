#include "IMUEmulator.h"
#include "IMUInterface.h"
#include "FailureInjector.h"

int main() {
    // Example setup
    IMUEmulator emulator(1000);
    FailureInjector injector;

    // Process some data (example)
    std::string trajectoryData = "example data";  // This should be replaced with real trajectory data
    emulator.processTrajectoryData(trajectoryData);
    std::string imuData = emulator.generateIMUData();
    if (!imuData.empty()) {
        imuData = injector.injectFailures(imuData);
        emulator.sendIMUData(imuData);
    }

    return 0;
}

