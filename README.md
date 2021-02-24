# office_bot
Project to set up a navigation system for an office robot.

Uses ROS Navigation stack for localisation, path planning and execution.

## Sub packages:

* office_bot: top level launch files
* office_bot_base: launch files for physical robot systems (the components which are not run when using gazebo)
* office_bot_description: urdf files describing the robot
* office_bot_gazebo: launch files to run office bot in simulation
* office_bot_navigation: launch/config for office bot nav stack
* office_bot_ui: for launching rviz + joystick control on remote machine

## TODO:

* add obstacles from bumper activation
* ability to patrol list of named locations
* add D435 obstacle detection