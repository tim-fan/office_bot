# THJC_BOT_BASE


Package to wrap serial control of a zumo-based robot in a ROS interface. 

The package is specific to controlling this robot:
https://github.com/thjc/can-hunter

The zumo base is running this sketch:
https://github.com/thjc/can-hunter/blob/master/slave_control/slave_control.cpp

That sketch listens to the serial port for motor speed commands, which are sent to the motors.

This package takes the MotorControl library defined [here](https://github.com/thjc/can-hunter/blob/master/can-hunter/jni/MotorControl.h), and wraps it in a ROS node which responds to twist commands.
