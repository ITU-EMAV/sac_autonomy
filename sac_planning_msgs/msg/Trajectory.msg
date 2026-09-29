# A short trajectory to drive, from near the car onwards, with a speed at each point (the
# local planner publishes one every cycle; the controller drops it when it gets old).
std_msgs/Header header   # frame: map; stamp: when it was planned
TrajectoryPoint[] points
bool stopping            # no way past: the speeds come down to 0 before an obstacle
