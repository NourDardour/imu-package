cmake_minimum_required(VERSION 3.10)

# Set the project name
project(IMUPackage)

# Add source files
add_executable(IMUPackage
    src/main.cpp
    src/IMUEmulator.cpp
    src/IMUInterface.cpp
    src/FailureInjector.cpp
)

# Tests
add_executable(tests
    tests/test_IMUEmulator.cpp
    tests/test_IMUInterface.cpp
    tests/test_FailureInjector.cpp
)

