# Error of a localization estimate against a ground truth (the simulation's exact pose), at
# the estimate's stamp.

std_msgs/Header header
float64 horizontal            # [m] distance in x-y
float64 vertical              # [m] estimate z - true z
float64 along_track           # [m] error along the direction the car faces (positive: ahead)
float64 cross_track           # [m] error to the car's left (positive: left)
float64 yaw                   # [rad] estimate yaw - true yaw
float64 horizontal_stddev     # [m] what the filter thinks its horizontal error is (1 sigma)
float64 yaw_stddev            # [rad] likewise for yaw
