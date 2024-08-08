
# IMU Package Design

## Overview

This project is an Inertial Measurement Unit (IMU) package emulator, designed to simulate IMU data for testing purposes. The package processes trajectory data and transforms it into IMU raw data, which can be sent to a device under test. The project is implemented in C++ and is structured to ensure modularity, scalability, and ease of testing.

## Features

- **IMU Emulation**: Processes and transforms trajectory data into IMU raw data.
- **Data Communication**: Interfaces with a device under test to send the generated IMU data.
- **Failure Injection**: Simulates potential failures in the IMU data stream for robust testing.
- **Modular Design**: Separate classes handle emulation, communication, and failure injection.

## Project Structure

```plaintext
IMU-Package-Design/
│
├── README.md
├── LICENSE (optional, but recommended)
├── docs/
│   ├── System_Overview.md
│   ├── System_Architecture.md
│   ├── Detailed_Design.md
│   └── Flow_of_Operations.md
├── src/
│   ├── IMUEmulator.cpp
│   ├── IMUEmulator.h
│   ├── IMUInterface.cpp
│   ├── IMUInterface.h
│   ├── FailureInjector.cpp
│   ├── FailureInjector.h
│   └── main.cpp
├── tests/
│   ├── test_IMUEmulator.cpp
│   ├── test_IMUInterface.cpp
│   ├── test_FailureInjector.cpp
│   └── CMakeLists.txt
└── .gitignore
```

## Getting Started

### Prerequisites

- A C++ compiler that supports C++11 or later.
- CMake (optional, for building the project and running tests).
- Git (to clone the repository).

### Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/IMU-Package-Design.git
   cd IMU-Package-Design
   ```

2. **Build the Project:**

   If you're using CMake:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

   Otherwise, compile manually:

   ```bash
   g++ src/*.cpp -o imu_package
   ```

### Usage

- **Run the Program:**

   After building the project, you can run the IMU package emulator with:

   ```bash
   ./imu_package
   ```

- **Running Tests:**

   To run the unit tests, compile and execute the test files in the `tests/` directory.

   ```bash
   g++ tests/test_IMUEmulator.cpp -o test_IMUEmulator
   ./test_IMUEmulator
   ```

   If using CMake, you can run all tests together:

   ```bash
   make tests
   ./tests
   ```


