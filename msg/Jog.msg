#--------------------------------------
#
# Herkulex package custom jog message
#
# used to send jog commands to a single servo.
#
# The target angle in radians is specified, then either the 
# time in seconds to complete the move or the angular rate
# in radians per second to complete the move.
#
# If both a specified then angular rate takes priority.
#
#--------------------------------------

string joint_name
float64 goal_angle
float64 goal_time
float64 goal_speed