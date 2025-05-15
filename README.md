# Camera Gimbal Stabilizer

A two-axis camera stabilization system based on STM32 microcontroller and brushless motors.

## Overview

This project implements a two-axis camera gimbal stabilizer using an STM32F401RE microcontroller, MPU6050 inertial sensor, and brushless motors with L6234PD drivers. The system provides camera stabilization for smooth footage during movement.

### Features

- Two-axis stabilization (pitch and roll)
- PID-based control system
- Real-time parameter adjustment via UART
- 3D printed components for mechanical assembly
- Modular design for future expansion
- Support for cameras up to 500g

## Hardware

### Components

- STM32F401RE Nucleo board
- MPU6050 accelerometer and gyroscope module
- 2× A2212 1000KV BLDC motors
- 2× L6234PD three-phase motor drivers
- 3D printed reduction gears
- 11.1V power supply for motors
- Breadboard, connection wires
- Plusivo Kit (sensors, jumpers, accessories)

### Connections

#### MPU6050 to STM32
- SDA → PB9
- SCL → PB8
- VCC → 3.3V
- GND → GND

#### UART for debugging:
- TX → PA2
- RX → PA3

#### STM32 to L6234PD (Motor Drivers)
- PWM1, PWM2, PWM3 → PA0, PA1, PA2 (X-axis Motor)
- PWM4, PWM5, PWM6 → PA3, PA4, PA5 (Y-axis Motor)
- EN1, EN2 → PB0, PB1 (Motors Enable)

## Software

The firmware is developed using STM32CubeIDE and leverages the following technologies:

- STM32 HAL for hardware abstraction
- I2C communication for MPU6050 sensor
- PWM generation for BLDC motor control
- PID control algorithms
- Complementary filter for sensor fusion
- UART for debugging and parameter adjustment

## Results

- Accurate orientation estimation with significantly reduced noise
- Stable angle measurement even under motion or vibration
- Real-time data reporting through UART interface
- Sampling rate of approximately 100Hz

## Project Status

- [x] 04/27/2025 - Component selection
- [x] 04/28/2025 - Mouser / eMAG order
- [x] 04/30/2025 - Order has arrived, except motors
- [x] 04/30/2025 - MPU6050 integration with STM32
- [x] 05/06/2025 - Motors expected to arrive
- [x] 05/14/2025 - Kalman filter implementation
- [ ] 05/06/2025 - PID control
- [ ] 05/07/2025 - Motor testing
- [ ] 05/10/2025 - Final test with physical assembly

## Future Improvements

- Add a third axis (yaw) for complete stabilization
- Implement object tracking capabilities
- Integrate a wireless control system

## Directory Structure

- Core/Src/ - Source code files
- Core/Inc/ - Header code files
- Hardware/ - Electrical schematics and hardware designs
- Models/ - 3D models for printed components
- Images/ - Project photos and diagrams

## Resources

### Hardware Resources
- [L6234PD Datasheet – STMicroelectronics](https://www.st.com/resource/en/datasheet/l6234.pdf)
- [MPU6050 – User Guide](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- [NUCLEO-F401RE – ST](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [A2212 1000KV Motor Specifications](https://www.emag.ro/motor-electric-brushless-outrunner-a2212-1400-kv-d28xl25mm-pentru-aeromodele-navomodele-si-drone-p2-0031-1400/pd/DNXV3LMBM/)

### Software Resources
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [PID control – Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [Madgwick's IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [Kalman filters for IMU processing](https://github.com/TKJElectronics/KalmanFilter)