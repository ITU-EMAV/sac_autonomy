# State of a localization filter, published after each update.

std_msgs/Header header
string estimator              # e.g. "ekf"
string motion_model           # e.g. "constant_acceleration"
bool initialized
float64[] position_stddev     # [m] x, y, z
float64 yaw_stddev            # [rad]
SensorStatus[] sensors
