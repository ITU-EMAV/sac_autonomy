#include "trajectory_planner.hpp"

namespace smart_car
{
  TrajectoryPlanner::TrajectoryPlanner(const rclcpp::NodeOptions &options)
      : Node("trajectory_planner_exe", options), tf_buffer(this->get_clock()), tf_listener(tf_buffer)
  {

    RCLCPP_INFO(this->get_logger(), "Trajectory planner node is started!");
    // initialize variables
    this->declare_parameter<double>("origin_pose._x", 0.0);
    this->declare_parameter<double>("origin_pose._y", 0.0);
    this->declare_parameter<std::string>("osm_path", "./map.osm");

    this->declare_parameter<std::string>("pose_topic", "/current_pose");
    this->declare_parameter<std::string>("path_topic", "/path_ids");
    this->declare_parameter<std::string>("trajectory_topic", "/trajectory");
    this->declare_parameter<std::string>("occupancy_grid_topic", "/occupancy_grid");
    this->declare_parameter<bool>("follow_centerline", true);
    this->declare_parameter<double>("curvature_contributing_factor", 0.3);
    this->declare_parameter<int>("clearance", 5);
    this->declare_parameter<double>("centerline_contributing_factor", 1.0);
    // Cost of a cell whose occupancy value is unknown (negative in the grid).
    this->declare_parameter<double>("unknown_cell_cost", 50.0);
    // Cost charged per unit of occupancy value for every traversable cell:
    // value 0 is free, and the cost grows linearly up to the lethal threshold.
    this->declare_parameter<double>("occupancy_cost_factor", 0.5);
    // Cells at or above this occupancy value are the only ones A* may not enter.
    this->declare_parameter<int>("lethal_cost_threshold", 100);
    // Cost charged per degree of heading change on a step.
    this->declare_parameter<double>("turn_penalty", 0.5);
    // Wall-clock budget for one A* search, in seconds. poseCallback runs on the
    // executor thread, so an unreachable goal must not stall the container.
    this->declare_parameter<double>("planning_time_budget", 0.05);
    this->declare_parameter<std::string>("switch_controller_topic", "/switch_controller");
    this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");

    // Which search runs once an in-lane obstacle takes the vehicle off the
    // centerline: "astar" searches the whole grid, "lattice" scores a fan of
    // laterally offset spline candidates.
    this->declare_parameter<std::string>("planner_type", "astar");

    // ---- Lattice planner --------------------------------------------------
    // Forward extent of the centerline window handed to the lattice, in meters.
    // The occupancy grid reaches grid_size * grid_resolution / 2 = 15 m ahead.
    this->declare_parameter<double>("lattice.planner_horizon", 15.0);
    // How far behind the vehicle centerline points are still kept, in meters.
    this->declare_parameter<double>("lattice.behind_distance", 3.0);
    // Route-index window around the closest centerline point. Bounds the search
    // so a route that loops back through the grid cannot capture the anchor.
    this->declare_parameter<int>("lattice.window_behind", 10);
    this->declare_parameter<int>("lattice.window_ahead", 100);
    // Speed range used to scale the anchor distance.
    this->declare_parameter<double>("lattice.min_speed", 1.0);
    this->declare_parameter<double>("lattice.max_speed", 2.0);
    // Anchor distance range. The lower bound also bounds candidate curvature:
    // a 3 m offset over 8 m needs about a 3.6 m radius, and the vehicle can do
    // 1.7 m wheelbase / tan(23 deg) = 4.0 m.
    this->declare_parameter<double>("lattice.min_lookahead", 8.0);
    this->declare_parameter<double>("lattice.max_lookahead", 12.0);
    // Spline sample spacing, in meters. Keep it below the grid resolution so a
    // collision check cannot step over an occupied cell.
    this->declare_parameter<double>("lattice.path_resolution", 0.2);
    // Lateral offsets of the candidates, in meters. The lane is about 2 m wide
    // and obstacles are inflated by ~1.85 m, so clearing a centred obstacle
    // needs roughly 2.5 m of offset.
    // Listed left-first within each magnitude: equal-cost candidates are broken
    // by list order, so a symmetric obstacle is passed on the overtaking side.
    this->declare_parameter<std::vector<double>>(
        "lattice.candidate_offsets",
        {0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 2.5, -2.5, 3.0, -3.0});
    // Cost per meter of lateral offset from the centerline.
    this->declare_parameter<double>("lattice.offset_weight", 1.0);
    // Cost, in equivalent meters of offset, of a candidate running entirely
    // over lethal-valued surface. Scales linearly with the occupancy value.
    this->declare_parameter<double>("lattice.off_lane_penalty", 6.0);
    this->declare_parameter<bool>("lattice.publish_debug", true);

    this->get_parameter("osm_path", this->osm_path);
    this->get_parameter("origin_pose._x", this->origin_x);
    this->get_parameter("origin_pose._y", this->origin_y);
    this->get_parameter("follow_centerline", this->follow_centerline);
    this->get_parameter("curvature_contributing_factor", this->curvature_contributing_factor);
    this->get_parameter("clearance", this->clearance);
    this->get_parameter("centerline_contributing_factor", this->centerline_contributing_factor);
    this->get_parameter("unknown_cell_cost", this->unknown_cell_cost);
    this->get_parameter("occupancy_cost_factor", this->occupancy_cost_factor);
    this->get_parameter("lethal_cost_threshold", this->lethal_cost_threshold);
    this->get_parameter("turn_penalty", this->turn_penalty);
    this->get_parameter("planning_time_budget", this->planning_time_budget);

    const std::string planner_type_name = this->get_parameter("planner_type").as_string();
    if (planner_type_name == "lattice")
    {
      this->planner_type = PlannerType::Lattice;
    }
    else
    {
      this->planner_type = PlannerType::AStar;
      if (planner_type_name != "astar")
      {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown planner_type '%s', falling back to 'astar'",
                    planner_type_name.c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Avoidance planner: %s",
                this->planner_type == PlannerType::Lattice ? "lattice" : "astar");

    this->get_parameter("lattice.planner_horizon", this->lattice_planner_horizon);
    this->get_parameter("lattice.behind_distance", this->lattice_behind_distance);
    this->get_parameter("lattice.window_behind", this->lattice_window_behind);
    this->get_parameter("lattice.window_ahead", this->lattice_window_ahead);
    this->get_parameter("lattice.publish_debug", this->lattice_publish_debug);

    this->lattice.setSpeedLimits(this->get_parameter("lattice.min_speed").as_double(),
                                 this->get_parameter("lattice.max_speed").as_double());
    this->lattice.setLookaheadDistances(
        this->get_parameter("lattice.min_lookahead").as_double(),
        this->get_parameter("lattice.max_lookahead").as_double());
    this->lattice.setPathResolution(this->get_parameter("lattice.path_resolution").as_double());
    this->lattice.setCandidateOffsets(
        this->get_parameter("lattice.candidate_offsets").as_double_array());
    this->lattice.setOffsetWeight(this->get_parameter("lattice.offset_weight").as_double());
    this->lattice.setOffLanePenalty(this->get_parameter("lattice.off_lane_penalty").as_double());
    // Both planners share one definition of "impassable".
    this->lattice.setLethalThreshold(this->lethal_cost_threshold);

    std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
    std::string PATH_TOPIC = this->get_parameter("path_topic").as_string();
    std::string TRAJECTORY_TOPIC = this->get_parameter("trajectory_topic").as_string();
    std::string OCCUPANCY_GRID_TOPIC = this->get_parameter("occupancy_grid_topic").as_string();
    std::string SWITCH_CONTROLLER_TOPIC = this->get_parameter("switch_controller_topic").as_string();
    std::string ODOMETRY_TOPIC = this->get_parameter("odometry_topic").as_string();

    // read lanelet map
    this->readLaneletMap();

    // initialize subscribers and publishers
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(POSE_TOPIC, rclcpp::QoS(1), std::bind(&TrajectoryPlanner::poseCallback, this, std::placeholders::_1));
    this->path_sub = this->create_subscription<sac_interfaces::msg::PathIds>(PATH_TOPIC, 10, std::bind(&TrajectoryPlanner::pathCallback, this, std::placeholders::_1));
    this->grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(OCCUPANCY_GRID_TOPIC, rclcpp::QoS(1), std::bind(&TrajectoryPlanner::occupancyGridCallback, this, std::placeholders::_1));
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>(TRAJECTORY_TOPIC, rclcpp::QoS(1));
    this->timing_pub = this->create_publisher<sac_interfaces::msg::PipelineTiming>(
        "/planning/trajectory/timing", rclcpp::QoS(1).best_effort());
    this->switch_controller_sub = this->create_subscription<std_msgs::msg::Bool>(SWITCH_CONTROLLER_TOPIC, 10, std::bind(&TrajectoryPlanner::switchControllerCallback, this, std::placeholders::_1));

    if (this->planner_type == PlannerType::Lattice)
    {
      // Only the lattice needs the speed, for its anchor distance.
      this->odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
          ODOMETRY_TOPIC, 10,
          std::bind(&TrajectoryPlanner::odometryCallback, this, std::placeholders::_1));
      if (this->lattice_publish_debug)
      {
        this->lattice_debug_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_planner/lattice_candidates", 10);
      }
    }
  }

  void TrajectoryPlanner::odometryCallback(const nav_msgs::msg::Odometry &msg)
  {
    this->current_speed = msg.twist.twist.linear.x;
  }

  void TrajectoryPlanner::switchControllerCallback(const std_msgs::msg::Bool &msg)
  {
    const bool next_follow_centerline = !msg.data;

    // The switch is republished at sensor rate; only report actual transitions.
    if (this->switch_controller_received && next_follow_centerline == this->follow_centerline)
    {
      return;
    }
    this->switch_controller_received = true;
    this->follow_centerline = next_follow_centerline;

    if (this->follow_centerline)
    {
      // Do not carry a committed lateral offset into the next avoidance run.
      this->lattice.resetSelection();
      RCLCPP_INFO(this->get_logger(), "Lane is clear: switching to centerline following");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected in the lane: switching to %s trajectory planning",
                  this->planner_type == PlannerType::Lattice ? "lattice" : "A*");
    }
  }

  void TrajectoryPlanner::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    plan_start_ = std::chrono::steady_clock::now();
    pose_stamp_ = rclcpp::Time(msg.header.stamp);

    if (!this->path || this->path->lanes.size() == 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No path!");
      return;
    }

    if (this->follow_centerline)
    {
      this->followCenterline(msg);
      return;
    }

    if (this->occupancy_grid.data.empty() ||
        this->occupancy_grid.info.resolution <= 0.0 ||
        this->occupancy_grid.info.width == 0 ||
        this->occupancy_grid.info.height == 0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Avoidance planning requested, but no valid occupancy grid is available");
      return;
    }

    std::vector<geometry_msgs::msg::PointStamped> centerline_point_ids;

    for (size_t i = 0; i < this->path->lanes.size(); i++)
    {
      const lanelet::ConstLanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[i]);
      for (auto point : current_lanelet.centerline())
      {
        geometry_msgs::msg::PointStamped point_;
        point_.header.frame_id = msg.header.frame_id; // we get the frame from the current pose which is mapped to the global frame
        point_.point.x = point.x();
        point_.point.y = point.y();
        point_.point.z = point.z();

        centerline_point_ids.push_back(point_);
      }
    }

    if (centerline_point_ids.size() == 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No centerline points!");
      return;
    }

    if (this->planner_type == PlannerType::Lattice)
    {
      this->planWithLattice(msg, centerline_point_ids);
    }
    else
    {
      this->planWithAStar(msg, centerline_point_ids);
    }
  }

  //--------------------------------------------------------------------------
  // A* avoidance planning: searches the local occupancy grid for a path to a
  // clear centerline point ahead, then smooths and publishes it.
  //--------------------------------------------------------------------------
  void TrajectoryPlanner::planWithAStar(
      const geometry_msgs::msg::PoseWithCovarianceStamped &msg,
      const std::vector<geometry_msgs::msg::PointStamped> &centerline_point_ids)
  {
    lanelet::BasicPoint2d current_point(msg.pose.pose.position.x, msg.pose.pose.position.y);
    const lanelet::Id current_lane_id =
        this->lanelet_map->laneletLayer.nearest(current_point, 1)[0].id();

    int goal_x = 0;
    int goal_y = 0;
    int goal_idx = 0;
    bool found = false;
    double distance = std::numeric_limits<double>::max();
    size_t current_idx = 0;

    // Find the nearest centerline point first.
    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      double current_x = msg.pose.pose.position.x;
      double current_y = msg.pose.pose.position.y;
      double new_distance = std::sqrt((centerline_point_ids[i].point.x - current_x) * (centerline_point_ids[i].point.x - current_x) +
                                      (centerline_point_ids[i].point.y - current_y) * (centerline_point_ids[i].point.y - current_y));
      if (new_distance < distance)
      {
        distance = new_distance;
        current_idx = i;
      }
    }

    double yaw = tf2::getYaw(msg.pose.pose.orientation);

    // Convert a map point into grid indices. Returns false when the point falls
    // outside the grid; the intermediate values are kept signed because a point
    // behind the grid origin produces negative offsets.
    auto toGrid = [&](const geometry_msgs::msg::Point &point, int &idx, int &idy) -> bool
    {
      const double dx = point.x - this->occupancy_grid.info.origin.position.x;
      const double dy = point.y - this->occupancy_grid.info.origin.position.y;

      const double rot_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
      const double rot_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);

      idx = static_cast<int>(std::floor(rot_x / this->occupancy_grid.info.resolution));
      idy = static_cast<int>(std::floor(rot_y / this->occupancy_grid.info.resolution));

      return idx >= 0 && idx < static_cast<int>(this->occupancy_grid.info.width) &&
             idy >= 0 && idy < static_cast<int>(this->occupancy_grid.info.height);
    };

    // Lane-following search parameters.
    const size_t LOOKAHEAD_POINTS = 20;
    const size_t MAX_SEARCH_RANGE = 20;

    size_t search_end = std::min(current_idx + MAX_SEARCH_RANGE, centerline_point_ids.size());

    for (size_t i = current_idx; i < search_end; i++)
    {
      int idx = 0;
      int idy = 0;
      if (!toGrid(centerline_point_ids[i].point, idx, idy))
      {
        continue;
      }

      const size_t grid_idx = static_cast<size_t>(idy) * this->occupancy_grid.info.width + idx;

      if (this->occupancy_grid.data[grid_idx] < this->lethal_cost_threshold)
      {
        goal_x = idx;
        goal_y = idy;
        goal_idx = i;
        found = true;

        if (i >= current_idx + LOOKAHEAD_POINTS)
        {
          break;
        }
      }
    }

    if (!found && search_end < centerline_point_ids.size())
    {
      size_t extended_end = std::min(search_end + 10, centerline_point_ids.size());

      for (size_t i = search_end; i < extended_end; i++)
      {
        int idx = 0;
        int idy = 0;
        if (!toGrid(centerline_point_ids[i].point, idx, idy))
        {
          continue;
        }

        const size_t grid_idx = static_cast<size_t>(idy) * this->occupancy_grid.info.width + idx;

        if (this->occupancy_grid.data[grid_idx] < this->lethal_cost_threshold)
        {
          goal_x = idx;
          goal_y = idy;
          goal_idx = i;
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No goal point found in the occupancy grid!, Current lane id: %ld and centerline idx: %ld, x: %lf, y: %lf", current_lane_id, current_idx, centerline_point_ids[current_idx].point.x, centerline_point_ids[current_idx].point.y);
      return;
    }

    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    double startHeading = yaw * RAD_TO_DEG;
    double goalHeading = startHeading;

    if (goal_idx > 0)
    {
      const double goal_dx = centerline_point_ids[goal_idx].point.x -
                             centerline_point_ids[goal_idx - 1].point.x;
      const double goal_dy = centerline_point_ids[goal_idx].point.y -
                             centerline_point_ids[goal_idx - 1].point.y;
      goalHeading = std::atan2(goal_dy, goal_dx) * RAD_TO_DEG;
    }

    AStar::Generator &generator = this->astar;
    generator.setWorldSize({static_cast<int>(this->occupancy_grid.info.width), static_cast<int>(this->occupancy_grid.info.height)});
    generator.clearCollisions();
    generator.clearTraversalCosts();
    generator.setCurvatureContributingFactor(this->curvature_contributing_factor);

    std::vector<Vec2i> centerline;
    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      int idx = 0;
      int idy = 0;
      if (!toGrid(centerline_point_ids[i].point, idx, idy))
      { // check if the point is in the grid
        continue;
      }
      centerline.push_back(Vec2i(idx, idy));
    }

    generator.setCenterlineContributingFactor(this->centerline_contributing_factor);
    generator.setTurnPenalty(this->turn_penalty);
    generator.setTimeBudget(this->planning_time_budget);
    generator.setCenterline(centerline);

    unsigned int start_x = this->occupancy_grid.info.width / 2;
    unsigned int start_y = this->occupancy_grid.info.height / 2;

    const unsigned int start_grid_idx = start_y * this->occupancy_grid.info.width + start_x;

    // Every cell below the lethal threshold stays traversable; its cost grows
    // linearly with the occupancy value, so value 0 is the preferred surface and
    // anything more expensive is only taken when it saves enough elsewhere.
    for (unsigned int y = 0; y < this->occupancy_grid.info.height; y++)
    {
      for (unsigned int x = 0; x < this->occupancy_grid.info.width; x++)
      {
        unsigned int idx = y * this->occupancy_grid.info.width + x;

        // The grid marks the start cell with a debug value; the vehicle is already
        // standing there, so it must stay free of both collision and cost.
        if (idx == start_grid_idx)
          continue;

        const int value = this->occupancy_grid.data[idx];

        if (value >= this->lethal_cost_threshold)
        {
          generator.addCollision({static_cast<int>(x), static_cast<int>(y)});
          continue;
        }

        const double cost = (value < 0) ? this->unknown_cell_cost
                                        : this->occupancy_cost_factor * value;
        if (cost > 0.0)
        {
          generator.setTraversalCost({static_cast<int>(x), static_cast<int>(y)}, cost);
        }
      }
    }

    Vec2i startCoord = {static_cast<int>(start_x), static_cast<int>(start_y)};

    generator.removeCollision({goal_x, goal_y});
    Vec2i goalCoord = {goal_x, goal_y};

    // RCLCPP_INFO(this->get_logger(), "Start: (%d, %d), Goal: (%d, %d)", start_x, start_y, goal_x, goal_y);

    std::vector<std::shared_ptr<AStar::Node>> path;
    int originalClearance = this->clearance;
    bool pathFound = false;
    for (int currentClearance = originalClearance; currentClearance >= 0; currentClearance--)
    {
      generator.setClearance(currentClearance);

      path = generator.findPath(startCoord, startHeading, goalCoord, goalHeading);
      if (!path.empty() && path.back()->coordinates == goalCoord)
      {
        pathFound = true;
        break;
      }
    }

    if (!pathFound)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path found!");
      return;
    }

    nav_msgs::msg::Path trajectory;
    for (auto node : path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = this->occupancy_grid.header.frame_id;

      // Step 1: Compute grid frame (local) position
      double rot_x = node->coordinates.x * this->occupancy_grid.info.resolution;
      double rot_y = node->coordinates.y * this->occupancy_grid.info.resolution;

      // Step 2: Rotate forward (undo the previous -yaw rotation)
      double dx = std::cos(yaw) * rot_x - std::sin(yaw) * rot_y;
      double dy = std::sin(yaw) * rot_x + std::cos(yaw) * rot_y;

      // Step 3: Translate using origin
      pose.pose.position.x = dx + this->occupancy_grid.info.origin.position.x;
      pose.pose.position.y = dy + this->occupancy_grid.info.origin.position.y;
      // Calculate orientation based on path direction
      if (trajectory.poses.size() > 0)
      {
        // Get direction from previous point to current point
        auto &prev_pose = trajectory.poses.back();
        double dx = pose.pose.position.x - prev_pose.pose.position.x;
        double dy = pose.pose.position.y - prev_pose.pose.position.y;
        double yaw = std::atan2(dy, dx);

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
      }
      else
      {
        // For first point, use default orientation
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
      }

      trajectory.poses.push_back(pose);
    }

    // Interpolate the path
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    for (auto pose : trajectory.poses)
    {
      x_coords.push_back(pose.pose.position.x);
      y_coords.push_back(pose.pose.position.y);
    }

    int window_size = 3;
    std::vector<double> x_smoothed = this->movingAverage(x_coords, window_size);
    std::vector<double> y_smoothed = this->movingAverage(y_coords, window_size);

    if (x_smoothed.size() < 3 || y_smoothed.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(), "Not enough points for spline interpolation. Publishing original path.");
      this->publishTrajectory(trajectory);
    }
    else
    {
      std::vector<double> distances;
      distances.push_back(0.0);
      for (size_t i = 1; i < x_smoothed.size(); ++i)
      {
        double dx = x_smoothed[i] - x_smoothed[i - 1];
        double dy = y_smoothed[i] - y_smoothed[i - 1];
        double dist = std::sqrt(dx * dx + dy * dy);
        distances.push_back(distances.back() + dist);
      }
      double total_distance = distances.back();

      std::vector<double> t;
      for (const auto &d : distances)
      {
        t.push_back(d / total_distance);
      }

      if (t.size() < 3)
      {
        RCLCPP_WARN(this->get_logger(), "Parameter vector t has insufficient points for spline interpolation.");
        this->publishTrajectory(trajectory);
      }
      else
      {
        const int num_fine = 50;
        std::vector<double> t_fine(num_fine);
        for (int i = 0; i < num_fine; ++i)
        {
          t_fine[i] = static_cast<double>(i) / (num_fine - 1);
        }
        tk::spline spline_x;
        tk::spline spline_y;
        spline_x.set_points(t, x_smoothed);
        spline_y.set_points(t, y_smoothed);
        std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
        for (const auto &ti : t_fine)
        {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = this->get_clock()->now();
          pose.header.frame_id = this->occupancy_grid.header.frame_id;
          pose.pose.position.x = spline_x(ti);
          pose.pose.position.y = spline_y(ti);
          pose.pose.position.z = 0.0;
          pose.pose.orientation.w = 1.0;
          interpolated_path.push_back(pose);
        }

        nav_msgs::msg::Path interpolated_path_msg;
        interpolated_path_msg.header.stamp = this->get_clock()->now();
        interpolated_path_msg.header.frame_id = this->occupancy_grid.header.frame_id;
        interpolated_path_msg.poses = interpolated_path;
        this->publishTrajectory(interpolated_path_msg);
      }
    }
  }

  //--------------------------------------------------------------------------
  // Lattice avoidance planning: spawns a fan of laterally offset spline
  // candidates around the centerline and publishes the cheapest clear one.
  //
  // Everything happens in the vehicle frame, which is also the occupancy
  // grid's frame: smart_car::OccupancyGrid rotates the grid by the vehicle yaw
  // and offsets its origin by half the grid, so the vehicle sits exactly on
  // the centre cell and grid x/y are vehicle forward/left.
  //--------------------------------------------------------------------------
  void TrajectoryPlanner::planWithLattice(
      const geometry_msgs::msg::PoseWithCovarianceStamped &msg,
      const std::vector<geometry_msgs::msg::PointStamped> &centerline_point_ids)
  {
    const double current_x = msg.pose.pose.position.x;
    const double current_y = msg.pose.pose.position.y;
    const double yaw = tf2::getYaw(msg.pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    // 1. Closest centerline point along the route.
    size_t closest_idx = 0;
    double min_distance_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < centerline_point_ids.size(); i++)
    {
      const double dx = centerline_point_ids[i].point.x - current_x;
      const double dy = centerline_point_ids[i].point.y - current_y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < min_distance_sq)
      {
        min_distance_sq = distance_sq;
        closest_idx = i;
      }
    }

    // 2. Route-index window around it, transformed into the vehicle frame. The
    //    index window matters: a route that loops back through the grid would
    //    otherwise offer points from the opposite direction as anchors.
    const size_t behind = static_cast<size_t>(std::max(0, this->lattice_window_behind));
    const size_t ahead = static_cast<size_t>(std::max(1, this->lattice_window_ahead));
    const size_t start_idx = (closest_idx > behind) ? closest_idx - behind : 0;
    const size_t end_idx = std::min(closest_idx + ahead, centerline_point_ids.size());

    std::vector<Lattice::Point> local_centerline;
    local_centerline.reserve(end_idx - start_idx);
    for (size_t i = start_idx; i < end_idx; i++)
    {
      const double dx = centerline_point_ids[i].point.x - current_x;
      const double dy = centerline_point_ids[i].point.y - current_y;
      // Global -> vehicle frame (rotate by -yaw).
      const double lx = dx * cos_yaw + dy * sin_yaw;
      const double ly = -dx * sin_yaw + dy * cos_yaw;
      if (lx > -this->lattice_behind_distance && lx < this->lattice_planner_horizon)
      {
        local_centerline.push_back({lx, ly});
      }
    }

    if (local_centerline.empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Lattice planner: no centerline points within the horizon");
      return;
    }

    // 3. Plan. The grid data is borrowed, so point the generator at it right
    //    before the search rather than caching it in the grid callback.
    this->lattice.setGridInfo(static_cast<int>(this->occupancy_grid.info.width),
                              static_cast<int>(this->occupancy_grid.info.height),
                              this->occupancy_grid.info.resolution,
                              this->occupancy_grid.data);
    this->lattice.setCenterline(std::move(local_centerline));

    const std::vector<Lattice::Point> best_local_path =
        this->lattice.computeTrajectory(this->current_speed);

    this->publishLatticeDebug(msg.header.frame_id, current_x, current_y, cos_yaw, sin_yaw);

    if (best_local_path.empty())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Lattice planner found no trajectory!");
      return;
    }

    // 4. Vehicle frame -> global frame. The Hermite splines are already smooth,
    //    so unlike the A* path this needs no moving average or respline.
    nav_msgs::msg::Path trajectory;
    trajectory.header.stamp = this->get_clock()->now();
    trajectory.header.frame_id = msg.header.frame_id;
    trajectory.poses.reserve(best_local_path.size());

    for (const auto &point : best_local_path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = trajectory.header;
      pose.pose.position.x = point.x * cos_yaw - point.y * sin_yaw + current_x;
      pose.pose.position.y = point.x * sin_yaw + point.y * cos_yaw + current_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      trajectory.poses.push_back(pose);
    }

    // Orientations from the path direction; the last pose reuses the previous
    // segment because it has no successor.
    for (size_t i = 0; i + 1 < trajectory.poses.size(); i++)
    {
      const double dx = trajectory.poses[i + 1].pose.position.x - trajectory.poses[i].pose.position.x;
      const double dy = trajectory.poses[i + 1].pose.position.y - trajectory.poses[i].pose.position.y;
      const double segment_yaw = std::atan2(dy, dx);
      trajectory.poses[i].pose.orientation.z = std::sin(segment_yaw / 2.0);
      trajectory.poses[i].pose.orientation.w = std::cos(segment_yaw / 2.0);
    }
    if (trajectory.poses.size() >= 2)
    {
      trajectory.poses.back().pose.orientation =
          trajectory.poses[trajectory.poses.size() - 2].pose.orientation;
    }

    this->publishTrajectory(trajectory);
  }

  void TrajectoryPlanner::publishLatticeDebug(
      const std::string &frame_id, double origin_x, double origin_y,
      double cos_yaw, double sin_yaw)
  {
    if (!this->lattice_debug_pub || this->lattice_debug_pub->get_subscription_count() == 0)
    {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    for (const auto &candidate : this->lattice.getAllTrajectories())
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "lattice_candidates";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;

      if (candidate.blocked)
      {
        marker.color.r = 1.0f;
        marker.color.a = 0.3f;
      }
      else
      {
        marker.color.g = 1.0f;
        marker.color.b = 0.5f;
        marker.color.a = 0.5f;
      }

      marker.points.reserve(candidate.points.size());
      for (const auto &point : candidate.points)
      {
        geometry_msgs::msg::Point p;
        p.x = point.x * cos_yaw - point.y * sin_yaw + origin_x;
        p.y = point.x * sin_yaw + point.y * cos_yaw + origin_y;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      markers.markers.push_back(std::move(marker));
    }

    this->lattice_debug_pub->publish(markers);
  }

  void TrajectoryPlanner::followCenterline(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    lanelet::BasicPoint2d current_point(msg.pose.pose.position.x, msg.pose.pose.position.y);

    size_t current_lane_idx = 0;
    size_t distance = std::numeric_limits<size_t>::max();

    for (size_t i = 0; i < this->path->lanes.size(); i++)
    {
      const lanelet::ConstLanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[i]);

      size_t new_distance = lanelet::geometry::distance2d(current_lanelet, current_point);

      if (new_distance < distance)
      {
        distance = new_distance;
        current_lane_idx = i;
      }
    }

    lanelet::Lanelet current_lanelet = this->lanelet_map->laneletLayer.get(this->path->lanes[current_lane_idx]);

    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = this->now();

    bool found = false;
    for (const auto &lane_id : this->path->lanes)
    {
      if (lane_id == current_lanelet.id())
        found = true; // Only follow the current lane
      if (!found)
        continue; // Skip lanes until we find the current lane

      lanelet::Lanelet lane = this->lanelet_map->laneletLayer.get(lane_id);
      for (const auto &point : lane.centerline())
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        trajectory.poses.push_back(pose);
      }
    }

    this->publishTrajectory(trajectory);
  }

  void TrajectoryPlanner::publishTrajectory(const nav_msgs::msg::Path & trajectory)
  {
    this->path_pub->publish(trajectory);
    sac_interfaces::msg::PipelineTiming timing;
    timing.stamp = this->get_clock()->now();
    timing.component = this->follow_centerline ? "trajectory_centerline" :
        (this->planner_type == PlannerType::Lattice ? "trajectory_lattice" : "trajectory_astar");
    timing.sequence = ++plan_sequence_;
    timing.execution_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_start_).count();
    timing.source_age_ms = (this->get_clock()->now() - pose_stamp_).seconds() * 1000.0;
    timing.processed_count = plan_sequence_;
    timing_pub->publish(timing);
  }

  void TrajectoryPlanner::pathCallback(const sac_interfaces::msg::PathIds &path_ids)
  {
    this->path = std::make_shared<sac_interfaces::msg::PathIds>(path_ids);
    // RCLCPP_INFO(this->get_logger(), "Path received!");
  }

  void TrajectoryPlanner::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid grid)
  {
    this->occupancy_grid = grid;
  }

  void TrajectoryPlanner::readLaneletMap()
  {
    lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});

    lanelet::projection::UtmProjector projector = lanelet::projection::UtmProjector(origin);

    this->lanelet_map = lanelet::load(this->osm_path, projector);

    if (this->lanelet_map == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
      rclcpp::shutdown(); // BE CAREFUL FOR CONTAINERIZED APPLICATIONS
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");
    }
  }

  std::vector<double> TrajectoryPlanner::movingAverage(const std::vector<double> &data, int windowSize)
  {
    std::vector<double> smoothed;
    int n = data.size();
    int halfWindow = windowSize / 2;

    for (int i = 0; i < n; i++)
    {
      double sum = 0.0;
      int count = 0;
      // pencereyi verinin başından ve sonundan taşmayacak şekilde ayarla
      for (int j = std::max(0, i - halfWindow); j < std::min(n, i + halfWindow + 1); j++)
      {
        sum += data[j];
        count++;
      }
      smoothed.push_back(sum / count);
    }
    return smoothed;
  }
} // namespace smart_car

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::TrajectoryPlanner)
