#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream> // For stringstream

// Class representing the raw IMU data
class IMURawData {
public:
    std::string imu_id;
    uint8_t status; // 8 bits for status
    float acceleration[3]; // X, Y, Z
    float gyro[3]; // Roll, Pitch, Yaw

    IMURawData(std::string id, uint8_t stat, float acc[], float gyr[]) 
        : imu_id(id), status(stat) {
            for (int i = 0; i < 3; ++i) {
                acceleration[i] = acc[i];
                gyro[i] = gyr[i];
            }
        }
};

// Class to handle trajectory data subscription
class TrajectorySubscriber {
public:
    TrajectorySubscriber(ros::NodeHandle& nh, IMUEmulator* emulator) {
        trajectory_subscriber = nh.subscribe("/trajectory_data", 1000, &TrajectorySubscriber::trajectoryCallback, this);
        this->emulator = emulator;
    }

private:
    ros::Subscriber trajectory_subscriber;
    IMUEmulator* emulator; // Pointer to IMUEmulator for passing trajectory data

    void trajectoryCallback(const std_msgs::Float64::ConstPtr& msg) {
        emulator->updateTrajectory(msg->data); // Updates the trajectory in the IMUEmulator
    }
};

// Class to handle sending IMU data
class IMUDataSender {
public:
    IMUDataSender(ros::NodeHandle& nh) {
        imu_data_publisher = nh.advertise<std_msgs::String>("/imu_data_interface", 1000);
    }

    void sendIMUData(const IMURawData& data) {
        std::stringstream msg;
        msg << data.imu_id << ", " << (int)data.status << ", "
            << data.acceleration[0] << ", " << data.acceleration[1] << ", " << data.acceleration[2] << ", "
            << data.gyro[0] << ", " << data.gyro[1] << ", " << data.gyro[2];
        
        std_msgs::String message;
        message.data = msg.str();
        imu_data_publisher.publish(message); // Publish the formatted message
    }

private:
    ros::Publisher imu_data_publisher; // Publisher for IMU data
};

// Class to manage IMU status and introduce failure simulation
class StatusHandler {
public:
    StatusHandler() : current_status(0) {} // Initialize status to normal operation

    void setStatus(uint8_t flag) {
        current_status = flag; // Update status based on the incoming flag
    }

    void introduceFailure() {
        // Logic to randomly introduce a failure or a specific type of failure
        if (/* condition to introduce failure */) {
            current_status = 1; // Example: Sensor fault
        } else {
            current_status = 0; // Normal operation
        }
    }

    uint8_t getCurrentStatus() {
        return current_status; // Return the current status
    }

private:
    uint8_t current_status; // Current operational state of the IMU
};

// Main IMU Emulator class
class IMUEmulator {
public:
    IMUEmulator(const std::string& id, int frequency, ros::NodeHandle& nh) 
        : imu_id(id), send_frequency(frequency) {
        trajectory_subscriber = new TrajectorySubscriber(nh, this);
        imu_data_sender = new IMUDataSender(nh);
        ros::Rate rate(send_frequency);
        
        while (ros::ok()) {
            processData();
            sendData();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void updateTrajectory(double data) {
        last_trajectory[1] = last_trajectory[0]; // Shift old data
        last_trajectory[0] = data; // Store the latest trajectory data
    }

private:
    ros::NodeHandle n; 
    TrajectorySubscriber* trajectory_subscriber; // Subscriber for trajectory data
    IMUDataSender* imu_data_sender; // Sender for IMU data
    std::string imu_id; // Identifier for the IMU
    int send_frequency; // Transmission frequency in Hz
    double last_trajectory[2]; // Latest two trajectory data points
    uint8_t imu_status = 0; // Normal operation by default

    void processData() {
        if (last_trajectory[0] != 0) { // Check if new data is available
            float acc[3]; // Placeholder for simulated acceleration
            float gyr[3]; // Placeholder for simulated gyro data
            convertToIMUData(last_trajectory, acc, gyr);
            imu_status = 0; // Sample simulation of status, this could be improved
        }
    }

    void convertToIMUData(const double trajectory[], float acc[], float gyr[]) {
        acc[0] = (trajectory[1] - trajectory[0]) * 9.81; // Mocked calculation for acceleration
        gyr[0] = 0.1; // Mocked gyro values
    }

    void sendData() {
        float acc[3] = {0.0, 0.0, 0.0}; // Replace with actual processed data
        float gyr[3] = {0.0, 0.0, 0.0}; // Replace with actual processed data
        IMURawData imu_data(imu_id, imu_status, acc, gyr);
        imu_data_sender->sendIMUData(imu_data); // Use IMUDataSender to send the data
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_emulator_node");
    ros::NodeHandle n;  // Create node handle for ROS
    IMUEmulator imu_emulator("IMU_1", 100, n); // Example initialization with node handle
    return 0;
}