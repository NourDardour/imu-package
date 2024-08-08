# IMU Package

## Overview

The **IMU Package** is a software solution designed to emulate an Inertial Measurement Unit (IMU) for testing purposes. It converts trajectory data into simulated IMU raw data, which is then used to validate the performance and accuracy of navigation systems by providing realistic IMU data to a board under test.

## Features

- **Trajectory Data Processing**: Transforms trajectory data into IMU raw data.
- **Data Transmission**: Sends simulated IMU data to a board under test.
- **Failure Injection**: Optionally introduces data anomalies to test robustness.
- **Modular Design**: Features clear separation of functionalities into distinct classes.
- **High-Frequency Handling**: Capable of processing data and updating at high frequencies.

## System Requirements

- **Python 3.x**: Ensure Python is installed on your system.
- **ROS (Robot Operating System)**: Required for handling trajectory data.
- **Optional Dependencies**: Libraries such as NumPy and SciPy for advanced functionality.

## Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/your-username/imu-package.git
   cd imu-package
   ```

2. **Install Dependencies**:

   Install Python dependencies listed in `requirements.txt`:

   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. **Configure the IMU Emulator**:

   Modify configuration files to set parameters for the IMU emulator according to your testing needs.

2. **Run the IMU Emulator**:

   Start the IMU emulator by running the main script:

   ```bash
   python src/imu_emulator.py
   ```

3. **Connect to the Board**:

   Ensure the board under test is properly connected and configured to receive data from the IMU emulator.

## Directory Structure

- `src/`: Contains source code for the IMU package.
  - `imu_emulator.py`: Main script for running the IMU emulator.
  - `imu_interface.py`: Manages communication with the board.
  - `failure_injector.py`: Handles failure injection into the IMU data stream.
- `tests/`: Contains unit tests for the IMU package components.
- `docs/`: Includes documentation and diagrams related to the IMU package.
- `requirements.txt`: Lists Python dependencies required for the project.
- `README.md`: This file.

## Code Examples

### IMUEmulator

```python
class IMUEmulator:
    def __init__(self, frequency):
        self.trajectory_data = []
        self.frequency = frequency
    
    def process_trajectory_data(self, data):
        """ Store and manage trajectory data. """
        self.trajectory_data.append(data)
        if len(self.trajectory_data) > 2:
            self.trajectory_data.pop(0)  # Keep only the latest two points
    
    def generate_imu_data(self):
        """ Transform the latest trajectory data into IMU raw data. """
        if len(self.trajectory_data) < 2:
            return None
        # Placeholder for transformation algorithm
        return transform_trajectory_to_imu(self.trajectory_data[-2:])
    
    def send_imu_data(self, imu_data):
        """ Send the IMU data to the board. """
        imu_interface = IMUInterface()
        imu_interface.send_to_board(imu_data)
```

### IMUInterface

```python
class IMUInterface:
    def send_to_board(self, imu_data):
        """ Send formatted IMU data to the board. """
        # Implement communication protocol
        pass
```

### FailureInjector

```python
class FailureInjector:
    def inject_failures(self, imu_data):
        """ Introduce failures into the IMU data for testing. """
        if some_condition:
            imu_data.status |= 0x01  # Set a specific error flag
        return imu_data
```

## Contributing

We welcome contributions to the IMU Package. To contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to your branch (`git push origin feature-branch`).
5. Create a Pull Request on GitHub.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or support, please contact:

- **Email**: your-email@example.com
- **GitHub Issues**: [Create an Issue](https://github.com/your-username/imu-package/issues)
```

### Instructions for Setup and Push to GitHub

1. **Initialize Git Repository**: If not already done, initialize a Git repository in your project directory:

   ```bash
   git init
   ```

2. **Add Remote Repository**: Add the remote repository URL (replace with your GitHub repository URL):

   ```bash
   git remote add origin https://github.com/your-username/imu-package.git
   ```

3. **Stage Changes**: Add files to the staging area:

   ```bash
   git add .
   ```

4. **Commit Changes**: Commit your changes with a descriptive message:

   ```bash
   git commit -m "Initial commit with IMU package code and documentation"
   ```

5. **Push Changes**: Push your changes to the GitHub repository:

   ```bash
   git push -u origin master
   ```

6. **Authenticate**: When prompted for a password, use your GitHub Personal Access Token (PAT) if you’ve set up two-factor authentication (2FA) or if GitHub requires it for authentication.
