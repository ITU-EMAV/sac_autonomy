uint8 POTHOLE=0
uint8 SPEED_BUMP=1
uint8 kind
float32 confidence
# Position in array header optical frame; invalid depth is never free space.
bool position_valid
geometry_msgs/Point position
float32 radius_m

# Original detector coordinates and metric Euclidean camera range.
uint16 model_class_id
float32[4] bbox_xyxy
float32 distance_m
# Only the speed planner sets path_assessed; false means UNKNOWN, not outside path.
bool path_assessed
bool intersects_expected_path
