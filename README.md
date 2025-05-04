# camera-gimbal-stabilizer
This project is a two-axis camera gimbal stabilizer. The modules interact as follows: MPU6050 provides orientation data to STM32 via I2C, STM32 processes the data through PID algorithms and generates PWM signals for the L6234PD drivers, which in turn control the brushless motors to stabilize the camera.
