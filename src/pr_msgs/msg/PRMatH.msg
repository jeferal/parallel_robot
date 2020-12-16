#Message to communicate long and multi-dimensional arrays, with header

std_msgs/Header header
builtin_interfaces/Time current_time

uint32 rows
uint32 cols

float64[<=140] data
