# How one sensor of the localization filter is doing, for tuning.

string name                   # the sensor's name in the config
string type                   # adapter type, e.g. "imu"
uint64 received               # messages since the start
uint64 accepted               # measurements fused
uint64 rejected               # measurements rejected as outliers
uint64 late                   # measurements older than the filter's history
float64 last_delay            # [s] receive time - message stamp
float64[] last_innovation     # z - h(x) of the last measurement
float64 last_mahalanobis      # its size in standard deviations
