# Differential Robot

## Parts
- DC Motor (without postition encoding) 2x (I took the motors out of old gamepads, that had a rumble feature)
- IMU (GY 521)
- ESP32
- LIPO 3.7V
- DC-DC converter (for 3.7V to 5V or 9V for motors)
- tb6612fng (motor driver)
- Cables

## Software
- Micro-ROS
- jrowbergs Mpu6050 library
- SparkFun TB6612FNG library

## Tasks
### First Problem to Solve
What, if we have 2 motors for our differential wheled robot, but their speed is not aligned?
We need to figure out a control mechanic to align both motors, such that we are able to drive straight forward or make turns as intended.
For that we use a PID controller to adjust the motor speed, to align with the given yaw, that the GY521 sensor is sensing


## Ideas
Here are ideas that come up when I am sleeping or in random moments
- IR LED for possible camera tracking
- Magnetometer for more precise direction control
- Kalman Filter for multi sensor integration
