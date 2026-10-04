# Traffic Sign Label Names
uint8 PEDESTRIANS_CROSSING = 0
uint8 PARKING = 1
uint8 ROUNDABOUT = 2
uint8 NO_ENTRY = 3
uint8 ONE_WAY_STRAIGHT = 4
uint8 PRIORITY_ROUTE = 5
uint8 NO_PARKING = 6
uint8 STOP = 7
uint8 DISABLED_PARKING = 8
uint8 ROUNDABOUT_WARNING = 9
uint8 END_OF_MOTORWAY = 10
uint8 MOTORWAY = 11
uint8 NO_RIGHT_TURN = 12
uint8 NO_LEFT_TURN = 13
uint8 TURN_RIGHT_AHEAD = 14
uint8 TURN_LEFT_AHEAD = 15
uint8 GO_STRAIGHT_OR_TURN_RIGHT = 16
uint8 GO_STRAIGHT_OR_TURN_LEFT = 17
uint8 TURN_RIGHT = 18
uint8 TURN_LEFT = 19
uint8 GREEN_LIGHT = 20
uint8 RED_LIGHT = 21
uint8 UNKNOWN = 22

# Fields for detection data
uint8 label         # Label of the detected traffic sign
float32 confidence  # Confidence of the detection
uint16[] bbox       # Location of the bounding box
float32 distance    # Distance from the ZED Stereo Camera

