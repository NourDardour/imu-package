#include "FailureInjector.h"
#include <cassert>

void testInjectFailures() {
    FailureInjector injector;
    std::string imuData = "IMU Data";
    std::string modifiedData = injector.injectFailures(imuData);
    assert(modifiedData.find("Error Flag Set") != std::string::npos);
}

int main() {
    testInjectFailures();
    return 0;
}

