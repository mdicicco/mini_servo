# mini_servo
Small ROS metapackage for viewing and controlling a simple mini hobby servo robot

The package is intended to allow basic control of the mini servo style robots often found on eBay and similar websites.  They are often described as "Mini Industrial Robotic Arm" or "ABB 6 Axis Robot Mechanical Arm" and often come with 3 large and 3 small hobby servos (although newer models appear to be metal with 6 large servos).

![Mini Servo Robot](/docs/mini_servo_robot.jpg "Example MiniServo Robot")

This metapackage contains one package for the robot URDF and another package for the files autogenerated by the MoveIt! setup wizard to launch moveit and plan with the robot.  Meshes were hand built in OnShape and are included just for gross visualization.

![Robot in RViz](/docs/robot_in_rviz.png "Mini Servo Robot in RViz")

Known issues:
* Kinematics are not perfect.  Changes will be coming soon.
* Joint limits also need work and might be flipped in direction

Coming soon:
* Arduino files for control of the servos
* Controller plugin to move the arm from a ROS computer

Huge thanks to IFL-CAMP/iiwa_stack which was used as a baseline for this.  There may be a few stray iiwa references in there that still need to be removed.
