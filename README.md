# ROS-Integrated Robotic Arm controller

This project demonstrates how to control multiple servos using an Arduino in a ROS (Robot Operating System) environment. The code uses the `rosserial` package for communication between the Arduino and ROS, allowing ROS nodes to control the servos by publishing messages.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Hardware Setup](#hardware-setup)
3. [Software Setup](#software-setup)
4. [Uploading the Arduino Code](#uploading-the-arduino-code)
5. [Running the ROS Node](#running-the-ros-node)
6. [Testing the System](#testing-the-system)
7. [License](#license)

## Prerequisites

Before starting, make sure you have the following installed:
- **Arduino IDE**: Used to upload code to the Arduino.
- **ROS Noetic** (or a compatible ROS version): Make sure it's installed and configured on your system.
- **`rosserial` package**: For ROS and Arduino communication. Install it using:
  ```bash
  sudo apt-get install ros-noetic-rosserial-arduino
  sudo apt-get install ros-noetic-rosserial
  ```
- **Adafruit PWM Servo Driver Library**: You can install this library using the Arduino Library Manager or from [Adafruit's GitHub repository](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library).

## Hardware Setup

### Required Components
- An Arduino board (e.g., Arduino Uno).
- Adafruit PCA9685 PWM Servo Driver board.
- 5V servos (at least 5 for this setup).
- Jumper wires for connections.

### Connections
1. Connect the Arduino's SDA and SCL pins to the PCA9685's SDA and SCL pins respectively.
2. Power the PCA9685 with an external 5V power source for the servos.
3. Connect the ground (GND) of the power source and the PCA9685 to the Arduino's GND.
4. Connect your servos to channels 0, 1, 2, 3, and 5 on the PCA9685.

## Software Setup

### 1. Arduino Code

Upload the Arduino code to your board using the Arduino IDE. Make sure you have the Adafruit library installed.

### Arduino Code
```cpp
#### use the code from repository
```

### 2. ROS Python Node

Create a Python script (e.g., `servo_controller.py`) in your ROS package:

```python
#### use the code from repository
```

Make the script executable:
```bash
chmod +x servo_controller.py
```

## Uploading the Arduino Code

1. Connect your Arduino to your computer via USB.
2. Open the Arduino IDE and paste the modified code.
3. Select the correct port and board type in the IDE.
4. Upload the code to the Arduino.

## Running the ROS Node

1. Launch the `rosserial` node:
   ```bash
   rosrun rosserial_python serial_node.py /dev/ttyUSB0
   ```
   Replace `/dev/ttyUSB0` with the actual serial port of your Arduino.
   
2. Run the ROS Python node:
   ```bash
   rosrun your_package_name servo_controller.py
   ```

## Testing the System

- The servos should move according to the angles published by the ROS node. You can modify the angles and delays in the Python script to test different behaviors.
- Monitor the ROS topics using `rostopic list` and `rostopic echo /servo0_angle` to see messages being published.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
