#include "FailureInjector.h"
#include <iostream>

std::string FailureInjector::injectFailures(const std::string& imuData) {
    std::string modifiedData = imuData;
    // Introduce failure conditionally
    if (true /* replace with actual condition */) {
        modifiedData += " | Error Flag Set";
    }
    return modifiedData;
}

