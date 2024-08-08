#include "IMUInterface.h"

void testSendToBoard() {
    IMUInterface interface;
    interface.sendToBoard("Test IMU Data");
    // Check output manually or with a more complex system
}

int main() {
    testSendToBoard();
    return 0;
}

