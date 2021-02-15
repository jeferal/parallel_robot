#Message to comunicate the state of the robot (joint_pos and joint_vel)

std_msgs/Header header
builtin_interfaces/Time current_time

pr_msgs/PRArray q  #Joint position
pr_msgs/PRArray q_vel  #Joint velocity
