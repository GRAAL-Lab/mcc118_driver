# MCC118 Driver

A ROS 2 package for acquiring analog measurements from a Measurement Computing MCC 118 DAQ HAT and publishing them on ROS topics.

This package reads the voltage values from the DAQ board, applies a scaling/calibration step, and publishes cable tension forces as a `geometry_msgs/WrenchStamped` message on a ROS topic for downstream processing.

## Overview

The driver is implemented as a ROS 2 node named `mcc118_node` and exposes a single executable:

- `mcc118_publisher`

The node initializes the MCC 118 board, configures a continuous analog input scan on two channels, reads samples at a target rate of 1000 Hz, and publishes the latest values periodically.

## Features

- ROS 2 integration using `ament_cmake`
- Support for Measurement Computing MCC 118 HAT devices
- Continuous analog acquisition from multiple channels
- Real-time publishing of force measurements
- `geometry_msgs/msg/WrenchStamped` publication for compatibility with robot force/tension pipelines

## Hardware Requirements

- Measurement Computing MCC 118 DAQ HAT
- Raspberry Pi or compatible Linux host with the MCC DAQ HAT software installed
- Access to the DAQ HAT shared libraries and headers in `/usr/local/include` and `/usr/local/lib`

This project expects the MCC DAQ HAT SDK / libraries to be installed on the machine before building.


## Dependencies

The package declares the following dependencies:

- `ament_cmake`
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `daqhats_utils` 
- `rosidl_default_generators`

The CMake configuration also links against the MCC DAQ HAT library:

```cmake
target_link_libraries(mcc118_publisher
  daqhats
)
```

## Build Instructions

1. Install ROS 2 and the required build tools.
2. Install the MCC DAQ HAT SDK / libraries for the MCC 118 board. https://github.com/mccdaq/daqhats#install-instructions 
3. Clone the package into a ROS workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/GRAAL-Lab/mcc118_driver.git
```

4. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select mcc118_driver
```

5. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Run the Driver

Start the publisher node:

```bash
ros2 run mcc118_driver mcc118_publisher
```

The node will:

- open the selected MCC 118 device,
- start a continuous analog input scan,
- read channel samples,
- publish the results to ROS topics.

## Published Topics

### 1. `/cable_tension`

Type: `geometry_msgs/msg/WrenchStamped`

This message publishes the same force information in a standard ROS wrench format.

```text
header
wrench.force.x
wrench.force.y
```

This is useful when consuming the data in higher-level robot or force-control systems.


## Calibration and Scaling

The publisher applies a scaling factor of `20.0` to the raw DAQ sample values before publishing:

```cpp
double measurement = 20.0 * read_buf[index + i];
```

It also writes a calibration coefficient for channel 1 during startup:

```cpp
double slope_n = 1.04517611026;
double offset_n = -94.253128;
```

These values are specific to the current setup and may need to be adjusted depending on the attached sensor and calibration procedure.

## Typical Workflow

1. Ensure the MCC 118 board is connected and recognized by the system.
2. Launch the node with `ros2 run mcc118_driver mcc118_publisher`.
3. Verify the node is publishing the desired topics using:

```bash
ros2 topic list
ros2 topic echo /cable_tension
```

## Notes

- The node is designed for continuous readings and uses a scan rate of approximately 1000 Hz.
- The code checks for DAQ buffer overrun and hardware overrun conditions and exits the loop if either occurs.
- The package currently uses a blocking `scanf` prompt before starting the scan; it may be useful to remove or replace this with a more ROS-friendly startup pattern in future revisions.

## License

This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.

## Acknowledgements

This package builds on the Measurement Computing DAQ HAT SDK and ROS 2 tooling.

## Quick Reference

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select mcc118_driver

# Source
source ~/ros2_ws/install/setup.bash

# Run
ros2 run mcc118_driver mcc118_publisher
```
