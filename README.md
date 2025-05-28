# Camera Gimbal Stabilizer

A two-axis camera stabilization system based on STM32 microcontroller and servo motors.

## Overview

This project implements a two-axis camera gimbal stabilizer using an STM32F401RE microcontroller, MPU6050 inertial sensor, and TowerPro MG996R servo motors with DC-DC power management. The system provides camera stabilization for smooth footage during movement.

### Features

- Two-axis stabilization (pitch and roll)
- Kalman filter-based sensor fusion
- Real-time parameter adjustment via UART
- 3D printed components for mechanical assembly
- Modular design for future expansion
- Support for lightweight cameras and mobile devices
- High-precision servo control with PWM

## Bill of Materials (BOM)

| Component | Quantity | Supplier | Price/Unit | Link | Datasheet |
|-----------|----------|----------|------------|------|-----------|
| NUCLEO-F401RE Development Board | 1 | Mouser Electronics | €13.13 | [Mouser](https://ro.mouser.com/ProductDetail/STMicroelectronics/NUCLEO-F401RE?qs=fK8dlpkaUMvGeToFJ6rzdA%3D%3D) | [STM32F401RE Datasheet](https://www.st.com/resource/en/data_brief/nucleo-c031c6.pdf) |
| MPU6050 Gyroscope/Accelerometer Module | 1 | Mouser Electronics | €8.55 | [Mouser](https://ro.mouser.com/ProductDetail/Olimex-Ltd/MOD-MPU6050?qs=SUpef6bDnvVsH%252Bq1tWOBKA%3D%3D) | [MPU6050 Datasheet](https://ro.mouser.com/datasheet/2/306/RM-MPU-60xxA_rev_4-736751.pdf) |
| TowerPro MG996R Servo Motor | 2 | TowerPro | RON 80 | [TowerPro](https://towerpro.com.tw/product/mg996r/) | [MG996R Specifications](https://towerpro.com.tw/product/mg996r/) |
| Module DC-DC Step Down LM2596S | 2 | Optimus Digital | 12.99 RON | [Optimus](https://www.optimusdigital.ro/en/adjustable-step-down-power-supplies/1109-lm2596-dc-dc-step-down-module-5a.html) | [Module Specifications](https://www.optimusdigital.ro/en/adjustable-step-down-power-supplies/1109-lm2596-dc-dc-step-down-module-5a.html) |
| Mini Breadboard | 2 | Mouser Electronics | €2.60 | [Mouser](https://ro.mouser.com/ProductDetail/OSEPP-Electronics/LS-00047?qs=w%2Fv1CP2dgqofvkXBf4F3MQ%3D%3D) | [LS-00047 Datasheet](https://www.osepp.com/accessories/components/162-ls-00047-solder-able-breadboard-mini) |
| Resistors Kit | 1 | Mouser Electronics | €10.84 | [Mouser](https://ro.mouser.com/ProductDetail/SparkFun/COM-10969?qs=WyAARYrbSnYDX0pYE0qQCg%3D%3D) | N/A |
| Plusivo Kit | 1 | Optimus Digital | RON 40 | [Optimus](https://www.optimusdigital.ro/en/kits/12026-plusivo-electronics-starter-kit-0721248990075.html?search_query=plusivo+kit&results=56) | N/A |

## Hardware

### Components

- STM32F401RE Nucleo board
- MPU6050 accelerometer and gyroscope module
- 2× TowerPro MG996R servo motors (high-torque metal gear)
- 2× LM2596S DC-DC step-down modules
- 3D printed mechanical structure
- 12V power supply for servo motors
- USB power for STM32 development board
- Breadboard, connection wires
- Plusivo Kit (sensors, jumpers, accessories)

### Servo Motor Specifications

The TowerPro MG996R servos provide:
- **Operating voltage**: 4.8V - 7.2V (optimized at 6V)
- **Torque**: 9.4 kg⋅cm (4.8V), 11 kg⋅cm (6V)
- **Speed**: 0.20 sec/60° (4.8V), 0.17 sec/60° (6V)
- **Control signal**: PWM (50Hz, 1-2ms pulse width)
- **Weight**: 55g each
- **Metal gear construction** for durability and precision

### Power Management

The LM2596S DC-DC modules provide:
- **Input voltage**: 7V - 35V (12V nominal)
- **Output voltage**: Adjustable 1.25V - 30V (set to 6V)
- **Output current**: Up to 2A continuous
- **Efficiency**: ~85%
- **Protection**: Over-current and thermal protection

### Connections

#### MPU6050 to STM32
- SDA → PB9 (I2C1_SDA)
- SCL → PB8 (I2C1_SCL)
- VCC → 3.3V
- GND → GND

#### UART for debugging:
- TX → PA2 (USART2_TX)
- RX → PA3 (USART2_RX)

#### STM32 to Servo Motors
- PWM1 → PA6 (TIM3_CH1) - X-axis Servo
- PWM2 → PA7 (TIM3_CH2) - Y-axis Servo

#### Power Distribution
- **STM32**: 5V via USB connection
- **MPU6050**: 3.3V from STM32 regulator
- **Servo Motors**: 6V from LM2596S modules
- **DC-DC Input**: 12V external power supply

## Software

The firmware is developed using STM32CubeIDE and leverages the following technologies:

### Core Technologies
- **STM32 HAL** for hardware abstraction
- **I2C communication** for MPU6050 sensor
- **PWM generation** for servo motor control (50Hz, 1-2ms pulse width)
- **Kalman filter** for optimal sensor fusion
- **UART** for debugging and parameter adjustment

### Key Algorithms
- **Sensor Processing**: Raw data acquisition and calibration
- **Kalman Filter**: Advanced sensor fusion for stable angle estimation
- **Servo Control**: Precise PWM-based positioning
- **Real-time Processing**: 100Hz update rate

### Implementation Features
- Custom MPU6050 driver optimized for gimbal applications
- Kalman filter with gyroscope bias estimation
- Servo control with safety limits and smooth movement
- Memory management with custom allocation
- Interrupt-based processing for consistent timing

## Mathematical Model

### Kalman Filter Implementation

The system uses a 2-state Kalman filter for optimal sensor fusion:

**State Vector:**
```
x_k = [θ_k, b_k]ᵀ
```
Where:
- θ_k = Angle estimate at time k
- b_k = Gyroscope bias estimate at time k

**Prediction Step:**
```
θ_{k|k-1} = θ_{k-1|k-1} + (ω_k - b_{k-1|k-1}) × Δt
b_{k|k-1} = b_{k-1|k-1}
```

**Update Step:**
```
θ_{k|k} = θ_{k|k-1} + K_0 × (θ_accel - θ_{k|k-1})
b_{k|k} = b_{k|k-1} + K_1 × (θ_accel - θ_{k|k-1})
```

**Tuning Parameters:**
- Q_angle = 0.001 (process noise for angle)
- Q_bias = 0.003 (process noise for bias)
- R_measure = 0.03 (measurement noise)

## Results

- **Accurate orientation estimation** with significantly reduced noise
- **Stable angle measurement** even under motion or vibration
- **Real-time data reporting** through UART interface
- **Sampling rate** of approximately 100Hz
- **Smooth servo control** with precise positioning
- **Effective power management** through DC-DC conversion

## Future Improvements

- Add a third axis (yaw) for complete 3-axis stabilization
- Implement object tracking capabilities using computer vision
- Integrate wireless control system (Bluetooth/WiFi)
- Add camera trigger synchronization
- Implement auto-calibration routines
- Add mobile app for remote control and parameter adjustment

## Directory Structure

```
├── Core/
│   ├── Src/           # Source code files
│   └── Inc/           # Header files
├── Hardware/          # Electrical schematics and PCB designs
├── Models/            # 3D models for printed components
├── Images/            # Project photos and diagrams
├── Documentation/     # Detailed project documentation
└── Tests/             # Unit tests and validation data
```

## Key Files

- **main.c** - Main application logic and system initialization
- **mpu6050.c/h** - MPU6050 sensor driver and communication
- **kalman.c/h** - Kalman filter implementation
- **servo.c/h** - Servo motor control and PWM management
- **stm32f4xx_hal_msp.c** - Hardware abstraction layer configuration

## Performance Characteristics

### Power Consumption
- **Control circuits**: ~50mA at 3.3V/5V
- **Servo motors**: 10mA idle, up to 1.5A per servo under load
- **Total system**: ~3.5A peak at 12V, ~0.1A idle

### Mechanical Specifications
- **Payload capacity**: Up to 300g (lightweight cameras/phones)
- **Stabilization range**: ±45° on both axes
- **Response time**: <50ms for correction
- **Positioning accuracy**: ±0.5° in static conditions

### Communication Specifications
- **I2C frequency**: 400kHz for sensor communication
- **PWM frequency**: 50Hz for servo control
- **UART baud rate**: 115200 for debugging
- **Update rate**: 100Hz for real-time performance

## Calibration Process

1. **Gyroscope Calibration**: Automatic bias calculation during startup
2. **Accelerometer Calibration**: Six-position calibration procedure
3. **Servo Calibration**: Center position and range adjustment
4. **Filter Tuning**: Parameter optimization for specific applications

## Usage Instructions

1. **Power Connection**: Connect 12V supply to DC-DC modules
2. **USB Connection**: Connect STM32 to PC for programming and debugging
3. **Sensor Mounting**: Ensure MPU6050 is rigidly mounted to camera platform
4. **Calibration**: Follow calibration procedure for optimal performance
5. **Operation**: System automatically stabilizes once initialized

## Troubleshooting

### Common Issues
- **Servo jitter**: Check power supply stability and PWM signal quality
- **Drift**: Recalibrate gyroscope bias or adjust Kalman filter parameters
- **Poor response**: Verify mechanical connections and servo mounting
- **Communication errors**: Check I2C connections and pull-up resistors

## Resources

### Hardware Resources
- [MPU6050 User Guide](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- [NUCLEO-F401RE Documentation](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [TowerPro MG996R Specifications](https://towerpro.com.tw/product/mg996r/)
- [LM2596S DC-DC Module](https://www.optimusdigital.ro/en/adjustable-step-down-power-supplies/1109-lm2596-dc-dc-step-down-module-5a.html)

### Software Resources
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Kalman Filter Theory](https://github.com/TKJElectronics/KalmanFilter)
- [Servo Control Principles](https://en.wikipedia.org/wiki/Servo_control)

### Academic References
- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
- Chui, C.K., Chen, G. (2017). "Kalman Filtering: With Real-Time Applications"
- Crisnapati, P.N., et al. (2023). "Enhancing Gimbal Stabilization Using DMP and Kalman Filter"
