#ifndef FAILUREINJECTOR_H
#define FAILUREINJECTOR_H

#include <string>

class FailureInjector {
public:
    std::string injectFailures(const std::string& imuData);
};

#endif

