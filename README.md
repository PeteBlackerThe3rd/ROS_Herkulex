ROS Herkulex driver package
=======

#### Summary
This is a ROS package to interface with a chain of Herkulex robot servos. It publishes the standard *joint_states*
topic and listens for custom jog messages on the *jog* topic. A MoveIt compatible trajectory action server will be
added in the near future but this initial commit is very minimal. 

### Authors
This ROS package was developed and tested by Pete Blacker and was based upon initial work by user [lukaszmitka](https://github.com/lukaszmitka/herkulex_servo_controller).
