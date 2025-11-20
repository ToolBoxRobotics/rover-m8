# rover-m8


## 1. System architecture

Hardware

- PC (ROS master)
  - Runs: roscore, RViz, Gazebo, MoveIt, navigation stack, teleop, OpenCV processing, logging.

- Raspberry Pi CM4 on rover
  - Runs: sensor/actuator interface nodes, robot_state_publisher, base control, nav stack “local” bits if desired.

- Arduino Mega #1 (“drive_mega”)

  - Interfaces:
    - 6× DRI0002 motor controllers via PCA9685 (PWM) + GPIO DIR pins.
    - 6× quadrature encoders (A/B).
    - 4× steering servos via PCA9685.
    - TCA9548A I²C mux with:
      - MPU6050 IMU
      - AHT20 temp/humidity
      - INA219 voltage/current
    - Talks with CM4 via rosserial over USB.

- Arduino Mega #2 (“arm_mega”)
  - 5× NEMA17 + A4988 (STEP/DIR/EN).
  - Limit switches for each joint.
  - rosserial over USB.

- Kinect RGB-D camera via USB on CM4/PC.
- YB-MVV21-V1 GPS via USB (NMEA serial) on CM4/PC.
- Joystick via USB on PC.
