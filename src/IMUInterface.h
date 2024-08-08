#ifndef IMUINTERFACE_H
#define IMUINTERFACE_H

#include <string>

class IMUInterface {
public:
    void sendToBoard(const std::string& imuData);
};

#endif

