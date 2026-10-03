# An object of the perception: a cluster of obstacle cells followed over time (sac_perception)

uint8 UNKNOWN = 0
uint8 PEDESTRIAN = 1    # by its size: up to 1 m across, 1-2.2 m high
uint8 VEHICLE = 2       # 2.5-6 m long
uint8 SMALL = 3         # lower than 0.8 m (a cone, a box)
uint8 STRUCTURE = 4     # larger than any object (a wall, a slope): not followed for speed

# Its shape, by its class (after Autoware's shape_estimation)
uint8 BOX = 0           # a vehicle: a rectangle fitted to its L shape (Zhang et al., IV 2017)
uint8 CYLINDER = 1      # a person, a small thing: the smallest circle around it (length = width
                        # = its diameter)
uint8 POLYGON = 2       # anything else: footprint (convex polygons, their union; a structure in
                        # pieces of 2 m, so a curved wall is not a box over the road)

uint32 id               # the same while the object is followed
uint8 classification
bool confirmed          # seen in enough scans to be trusted
bool moving             # its speed is over the threshold and trusted

geometry_msgs/Point position      # its box's centre on the ground, in the header's frame
geometry_msgs/Vector3 velocity    # [m/s]
float32[4] position_covariance    # [m^2] xx, xy, yx, yy
float32[4] velocity_covariance    # [m^2/s^2]
float32 yaw                       # [rad] its box's long side
float32 length                    # [m] its box, along yaw
float32 width                     # [m]
float32 height                    # [m] its top over the ground
uint8 shape
geometry_msgs/Polygon[] footprint # with POLYGON, in the header's frame
float32 age                       # [s] since it was first seen
uint32 hits                       # scans it was seen in
