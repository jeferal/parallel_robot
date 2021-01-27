#Message used for the motion capture system of the robot

std_msgs/Header header
builtin_interfaces/Time current_time

pr_msgs/PRArray x_coord  #Coordinates detected by optitrack
float64 error            #Error between model and mocap
float64 latency          #Latency of the communication with optitrack
