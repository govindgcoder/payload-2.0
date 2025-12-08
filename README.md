# Payload 2.0

ESP32 as module hub for bme280, SD card, ESP-CAM and LoRa, incharge of communication and logging.
STM32 houses GPS and IMU, gets altitude from esp and calculates orientation and position.



## STM32

* powerhouse.



> madgwick + EKF 6dof was chosen instead of a combined large EKF to reduce complexity.



### Madgwick filter:

* used for calculating the orientation.



\### Five main steps for the updation algorithm

1. normalize measured values.
2. calculate gradient(s) based on accelerometer values.
3. prediction based on gyroscope values $q˙​gyro​=0.5×q⊗ω$.
4. fuse the data: $q˙​final​=q˙​gyro​−β×s$, $qnew​=q\_old​+q˙​final​×dt$, where dt is 1/sample frequency.
5. normalize the resultant quaternion.

repeat.



> using auxiliary variables to avoid repeated arithmetic is a great optimization.

