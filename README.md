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

## Bill of Materials (BOM)

| Component | Quantity | Supplier | Price/Unit | Link | Datasheet |
|-----------|----------|----------|------------|------|-----------|
| NUCLEO-F401RE Development Board | 1 | Mouser Electronics | €13.13 | [Mouser](https://ro.mouser.com/ProductDetail/STMicroelectronics/NUCLEO-F401RE?qs=fK8dlpkaUMvGeToFJ6rzdA%3D%3D) | [STM32F401RE Datasheet](https://www.st.com/resource/en/data_brief/nucleo-c031c6.pdf) |
| MPU6050 Gyroscope/Accelerometer Module | 1 | Mouser Electronics | €8.55 | [Mouser](https://ro.mouser.com/ProductDetail/Olimex-Ltd/MOD-MPU6050?qs=SUpef6bDnvVsH%252Bq1tWOBKA%3D%3D) | [MPU6050 Datasheet](https://ro.mouser.com/datasheet/2/306/RM-MPU-60xxA_rev_4-736751.pdf) |
| A2212 1000KV Brushless Motor | 2 | eMAG | RON 80 | [eMAG](https://www.emag.ro/motor-electric-brushless-outrunner-a2212-1000-kv-d28xl25mm-pentru-aeromodele-navomodele-si-drone-p2-0031-1000/pd/D0XV3LMBM/) | [A2212 Specifications](https://www.emag.ro/motor-electric-brushless-outrunner-a2212-1000-kv-d28xl25mm-pentru-aeromodele-navomodele-si-drone-p2-0031-1000/pd/D0XV3LMBM/#description-section/) |
| L6234PD Motor Driver | 3 | Mouser Electronics | €4.79 | [Mouser](https://ro.mouser.com/ProductDetail/STMicroelectronics/L6234PD?qs=lgHKUCmDFtgFRXXnpwFpNg%3D%3D) | [L6234PD Datasheet](https://ro.mouser.com/datasheet/2/389/l6234-1849406.pdf) |
| Mini Breadboard | 2 | Mouser Electronics | €2.60 | [Mouser](https://ro.mouser.com/ProductDetail/OSEPP-Electronics/LS-00047?qs=w%2Fv1CP2dgqofvkXBf4F3MQ%3D%3D) | [LS-00047 Datasheet](https://www.osepp.com/accessories/components/162-ls-00047-solder-able-breadboard-mini) |
| Resistors Kit | 1 | Mouser Electronics | €10.84 | [Mouser](https://ro.mouser.com/ProductDetail/SparkFun/COM-10969?qs=WyAARYrbSnYDX0pYE0qQCg%3D%3D) | N/A |

**Invoice**: [Mouser Invoice](https://ro.mouser.com/account/invoices/detail/84301659?hashId=mn6wOP)

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

# Kalman Filter Mathematical Model

The implemented Kalman filter follows a standard discrete-time approach for orientation estimation.

## State Space Representation

The filter uses a 2×1 state vector:

$$x_k = \begin{bmatrix} \theta_k \\ b_k \end{bmatrix}$$

Where:
- $\theta_k$ - Angle estimate at time k
- $b_k$ - Gyroscope bias estimate at time k

## Prediction Step

The state prediction equation:

$$\hat{x}_{k|k-1} = F_k \cdot \hat{x}_{k-1|k-1} + B_k \cdot u_k$$

In this implementation:

$$\hat{\theta}_{k|k-1} = \hat{\theta}_{k-1|k-1} + (\omega_k - \hat{b}_{k-1|k-1}) \cdot \Delta t$$
$$\hat{b}_{k|k-1} = \hat{b}_{k-1|k-1}$$

Where:
- $\omega_k$ - Angular rate from gyroscope
- $\Delta t$ - Time step

The error covariance is projected ahead:

$$P_{k|k-1} = F_k \cdot P_{k-1|k-1} \cdot F_k^T + Q_k$$

In the implementation:

$$P_{00} += \Delta t \cdot (\Delta t \cdot P_{11} - P_{01} - P_{10} + Q_{angle})$$
$$P_{01} -= \Delta t \cdot P_{11}$$
$$P_{10} -= \Delta t \cdot P_{11}$$
$$P_{11} += Q_{bias} \cdot \Delta t$$

## Update Step

The Kalman gain is calculated:

$$K_k = P_{k|k-1} \cdot H_k^T \cdot (H_k \cdot P_{k|k-1} \cdot H_k^T + R_k)^{-1}$$

In this implementation:

$$S = P_{00} + R_{measure}$$
$$K_0 = P_{00} / S$$
$$K_1 = P_{10} / S$$

The state is updated with the measurement:

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \cdot (z_k - H_k \cdot \hat{x}_{k|k-1})$$

In this implementation:

$$y = \theta_{accel} - \hat{\theta}_{k|k-1}$$
$$\hat{\theta}_{k|k} = \hat{\theta}_{k|k-1} + K_0 \cdot y$$
$$\hat{b}_{k|k} = \hat{b}_{k|k-1} + K_1 \cdot y$$

Where:
- $\theta_{accel}$ - Angle calculated from accelerometer

Finally, the error covariance is updated:

$$P_{k|k} = (I - K_k \cdot H_k) \cdot P_{k|k-1}$$

In this implementation:

$$P_{00} -= K_0 \cdot P_{00}$$
$$P_{01} -= K_0 \cdot P_{01}$$
$$P_{10} -= K_1 \cdot P_{00}$$
$$P_{11} -= K_1 \cdot P_{01}$$

## Tuning Parameters

The filter performance depends on three key parameters:

- $Q_{angle}$ - Process noise for angle (set to 0.001 in the implementation)
- $Q_{bias}$ - Process noise for bias (set to 0.003 in the implementation)
- $R_{measure}$ - Measurement noise (set to 0.03 in the implementation)

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
