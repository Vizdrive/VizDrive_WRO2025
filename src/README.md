# Souce Code Directory

Here you can find all the .ino files for the operation of our robot in the Open and Obstacle Challenges.
As well, you can find the test codes we used throughout the process of construction and debugging, besides the codes for data recolection and analysis for the ultrasonic sensors and the MPU6050.

The directory is organized in the following way:

## `src/computer_vision`

You will find the test code for the Pixycam 2.1 artificial vision and color signature detection. 

## `src/main_control`

You'll find the updated main code for the WRO competition challenges.

When you refer to the main source code, please consider that it is updated throughout the competition to improve or add different features, so it may vary between rounds.
In particular, the PID control constants, the amount of correction, and the different thresholds are modified depending on the performance.
You can see progress and changes over time by viewing previous versions in Github commits.

## `src/mpu_orientation_control`

You will find the test code and data recolection code for the MPU6050 gyroscope + accelerometer.

## `src/ultrasonic_sensors`

You will find the test code for raw measurements, the test code for filtered measurements, and the code for data recolection to compare how the error dispersion varies between raw and filtered measurements.

## `src/test_code`

You will find the general test codes for other functionalities as the color sensors suite, mainly to adjust the thresholds, or a code to test the motor and IR encoder.
