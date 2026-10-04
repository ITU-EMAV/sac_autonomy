# Sorting Order
int8 LEFT2RIGHT = -1
int8 RIGHT2LEFT = 1

std_msgs/Header header          # ROS standard message header
CameraDetection[] detections    # Detections array
string sort_order               # Sorting order of the detections
