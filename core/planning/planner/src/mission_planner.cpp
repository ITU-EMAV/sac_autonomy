#include "mission_planner.hpp"

// TODO:
// Switch planning strategy kısmında a star çalışması triggerlanırken ani olmalı ama dönerken yavaş olmalı şu anda fena değil ama geliştirilmeli
// Park kısmını kontrol edemedim park levhaları simülasyona yüklenmedi gerekirse blender ile model oluşutulup aktarılacak
// Kodlar temize çekilecek
namespace smart_car
{
  MissionPlanner::MissionPlanner(const rclcpp::NodeOptions &options)
      : Node("mission_planner_exe", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "mission_planner is initialized!");

    // ========================= PARAMETER DECLARATIONS =========================
    this->declare_parameter<double>("origin_pose._x", 0.0);
    this->declare_parameter<double>("origin_pose._y", 0.0);
    this->declare_parameter<std::string>("osm_path", "./map.osm");

    // Topic parameters
    this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
    this->declare_parameter<std::string>("pose_topic", "pose");
    this->declare_parameter<std::string>("path_topic", "path");
    this->declare_parameter<std::string>("stop_topic", "stop");
    this->declare_parameter<std::string>("yolo_topic", "yolo_detections");
    this->declare_parameter<bool>("enable_stop_sign", true);
    this->declare_parameter<bool>("enforce_freshness", true);
    this->declare_parameter<std::string>("telemetry_topic", "telemetry");

    // Semantic parameters
    this->declare_parameter<std::string>("station_name", "station");
    this->declare_parameter<std::string>("park_name", "park");
    this->declare_parameter<std::string>("switch_to_centerline_topic", "switch_to_centerline");
    this->declare_parameter<std::string>("no_entry_topic", "no_entry_ids");
    this->declare_parameter<std::string>("cluster_topic", "bottom_points");
    this->declare_parameter<std::string>("occupancy_grid_topic", "occupancy_grid");

    // ========================= PARAMETER RETRIEVAL =========================
    this->osm_path = this->get_parameter("osm_path").as_string();
    this->origin_x = this->get_parameter("origin_pose._x").as_double();
    this->origin_y = this->get_parameter("origin_pose._y").as_double();

    std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
    std::string GOAL_POSE_TOPIC = this->get_parameter("goal_pose_topic").as_string();
    std::string PATH_TOPIC = this->get_parameter("path_topic").as_string();
    std::string STOP_TOPIC = this->get_parameter("stop_topic").as_string();
    std::string YOLO_TOPIC = this->get_parameter("yolo_topic").as_string();
    std::string TELEMETRY_TOPIC = this->get_parameter("telemetry_topic").as_string();
    this->STATION_NAME = this->get_parameter("station_name").as_string();
    std::string PARK_NAME = this->get_parameter("park_name").as_string();
    std::string SWITCH_TO_CENTERLINE_TOPIC = this->get_parameter("switch_to_centerline_topic").as_string();
    std::string NO_ENTRY_TOPIC = this->get_parameter("no_entry_topic").as_string();
    std::string CLUSTER_TOPIC = this->get_parameter("cluster_topic").as_string();
    std::string OCCUPANCY_GRID_TOPIC = this->get_parameter("occupancy_grid_topic").as_string();

    // Print all parameters
    RCLCPP_INFO(this->get_logger(), "======================================");
    RCLCPP_INFO(this->get_logger(), "origin_pose._x: %.2f", this->origin_x);
    RCLCPP_INFO(this->get_logger(), "origin_pose._y: %.2f", this->origin_y);
    RCLCPP_INFO(this->get_logger(), "osm_path: %s", this->osm_path.c_str());
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: %s", GOAL_POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "pose_topic: %s", POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "path_topic: %s", PATH_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "stop_topic:  %s", STOP_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "yolo_topic: %s", YOLO_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "telemetry_topic: %s", TELEMETRY_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "station_name: %s", this->STATION_NAME.c_str());
    RCLCPP_INFO(this->get_logger(), "park_name: %s", PARK_NAME.c_str());
    RCLCPP_INFO(this->get_logger(), "cluster_topic: %s", CLUSTER_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "occupancy_grid_topic: %s", OCCUPANCY_GRID_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "switch_to_centerline_topic: %s", SWITCH_TO_CENTERLINE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "no_entry_topic: %s", NO_ENTRY_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "======================================");

    // ========================= MAP AND GRAPH INITIALIZATION =========================
    this->readLaneletMap();
    this->load_the_graph();

    // ========================= MEMBER VARIABLE INITIALIZATION =========================
    this->goal_pose_id = 0;
    this->vehicle_speed_ptr = nullptr;
    this->vehicle_speed_update_time = this->get_clock()->now();

    // ========================= SUBSCRIBER INITIALIZATION =========================
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        POSE_TOPIC, rclcpp::QoS(1), std::bind(&MissionPlanner::poseCallback, this, std::placeholders::_1));

    this->goal_sub = this->create_subscription<std_msgs::msg::Int32>(
        GOAL_POSE_TOPIC, 10, std::bind(&MissionPlanner::goalPoseCallback, this, std::placeholders::_1));

    this->yolo_sub = this->create_subscription<sac_interfaces::msg::CameraDetectionArray>(
        YOLO_TOPIC, rclcpp::QoS(1), std::bind(&MissionPlanner::yoloCallback, this, std::placeholders::_1));

    this->telemetry_sub = this->create_subscription<sac_interfaces::msg::Telemetry>(
        TELEMETRY_TOPIC, 10, std::bind(&MissionPlanner::telemetryCallback, this, std::placeholders::_1));

    this->cluster_sub = this->create_subscription<cluster_msgs::msg::ClusterPointsArray>(
        CLUSTER_TOPIC, rclcpp::QoS(1), std::bind(&MissionPlanner::cluster_callback, this, std::placeholders::_1));

    this->occupancy_grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        OCCUPANCY_GRID_TOPIC, rclcpp::QoS(1), std::bind(&MissionPlanner::occupancyGridCallback, this, std::placeholders::_1));

    // ========================= QOS PROFILES =========================
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.transient_local().reliable();

    // ========================= PUBLISHER INITIALIZATION =========================
    // Standard publishers
    this->park_pub_ = this->create_publisher<std_msgs::msg::Int32>("/park_id", 10);
    this->traffic_light_pub_ = this->create_publisher<std_msgs::msg::Bool>("/traffic_light", 10);
    this->behavioral_pub_ = this->create_publisher<std_msgs::msg::Int32>("/behavioral", 10);
    this->switch_controller_pub = this->create_publisher<std_msgs::msg::Bool>(SWITCH_TO_CENTERLINE_TOPIC, 10);
    this->stop_pub = this->create_publisher<std_msgs::msg::Bool>("/stop", 10);
    this->stop_distance_pub = this->create_publisher<std_msgs::msg::Float64>(STOP_TOPIC, 10);

    // Transient local publishers
    this->NoEntryPub = this->create_publisher<std_msgs::msg::Int32MultiArray>(NO_ENTRY_TOPIC, qos_profile);
    this->path_pub = this->create_publisher<sac_interfaces::msg::PathIds>(PATH_TOPIC, qos_profile);

    // ========================= ADDITIONAL INITIALIZATION =========================
    this->get_station_ids(); // Read station IDs from map
    clock_ = this->get_clock();
  }

  /**
   * @brief Callback function for processing pose messages with covariance information
   *
   * This Callback stores the pose data and finds the corresponding lanelet in the shortest path.
   * Also In this Callback we decide for planning method with respect to switchPlanningMode
   * This Function publishes shortest path's lane ids and distance to last position on the shortest path.
   *
   * @param msg The pose message containing pose data from the GAE Localization system
   */
  void MissionPlanner::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {

    this->current_position = msg.pose.pose.position;

    this->pose = msg.pose.pose;

    lanelet::BasicPoint2d curr_pt(this->current_position.x, this->current_position.y);
    this->is_pose_start = true;
    if (!this->lanelet_map || !this->routing_graph)
    {
      RCLCPP_WARN(this->get_logger(), "Lanelet map or routing graph not initialized yet");
      return;
    }
    if (!this->first_time)
    {
      lanelet::BasicPoint2d curr_pt(this->current_position.x, this->current_position.y);
      this->current_lanelet = this->lanelet_map->laneletLayer.nearest(curr_pt, 1)[0];
      this->previous_lanelet = this->current_lanelet;
    }
    else
    {
      if (this->first_time && !this->shortest_path.empty())
      {
        lanelet::BasicPoint2d query_point(this->current_position.x, this->current_position.y);
        const lanelet::ConstLanelet *nearest_ll = nullptr;
        double min_d = std::numeric_limits<double>::max();

        for (const auto &ll : this->shortest_path)
        {
          double d = lanelet::geometry::distance2d(ll, query_point);
          if (d < min_d)
          {
            min_d = d;
            nearest_ll = &ll;
          }
        }

        if (nearest_ll)
        {
          lanelet::Lanelet nearest_mut = this->lanelet_map->laneletLayer.get(nearest_ll->id());
          this->current_lanelet = nearest_mut;
          this->previous_lanelet = this->current_lanelet; // Yol üstünde kaldığımızdan senkron tut
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Could not find a nearest lanelet on shortest_path.");
        }
      }
    }

    BoostPoint current_point(curr_pt.x(), curr_pt.y());

    this->path_pub->publish(this->path_ids);
    {
      // // RCLCPP_INFO(this->get_logger(), "Size of path_ids: %zu", this->path_ids.lanes.size());

      // // === Normal stop distance hesaplama ===
      // if (!this->centerline_points.empty())
      // {
      //   size_t current_centerpoint_idx = 0;
      //   double distance_to_point = std::numeric_limits<double>::max();

      //   for (size_t i = this->previous_centerpoint_idx; i < this->centerline_points.size(); ++i)
      //   {
      //     double dist = std::hypot(
      //         this->centerline_points[i].x() - current_point.x(),
      //         this->centerline_points[i].y() - current_point.y());
      //     if (dist < distance_to_point)
      //     {
      //       distance_to_point = dist;
      //       current_centerpoint_idx = i;
      //     }
      //   }

      //   this->previous_centerpoint_idx = current_centerpoint_idx;
      //   current_centerpoint_idx = std::min(current_centerpoint_idx + 1, this->centerline_points.size() - 1);

      //   double distance = std::hypot(
      //       this->centerline_points[current_centerpoint_idx].x() - current_point.x(),
      //       this->centerline_points[current_centerpoint_idx].y() - current_point.y());

      //   for (size_t i = current_centerpoint_idx; i < this->centerline_points.size() - 1; ++i)
      //   {
      //     distance += std::hypot(
      //         this->centerline_points[i + 1].x() - this->centerline_points[i].x(),
      //         this->centerline_points[i + 1].y() - this->centerline_points[i].y());
      //   }

      //   // std_msgs::msg::Float64 stop_distance;
      //   // stop_distance.data = distance;
      //   // this->stop_distance_pub->publish(stop_distance);
      // }
      // else
      // {
      //   if (this->goal_pose_id != 0)
      //   {
      //     std_msgs::msg::Int32 goal_pose_id_msg;
      //     goal_pose_id_msg.data = this->goal_pose_id;
      //     this->goalPoseCallback(goal_pose_id_msg);
      //   }
      // }
    }
    this->switchPlanningMode();

    if (this->shortest_path.empty())
    {
      // RCLCPP_ERROR(this->get_logger(), "No path available! Skipping route length computation.");
      return;
    }

    double total_length = 0.0;
    auto route = routing_graph->getRoute(this->current_lanelet, this->shortest_path.back(), false);

    if (route)
    {
      auto path = route->shortestPath();
      for (const auto &ll : path)
      {
        auto centerline_2d = lanelet::utils::to2D(ll.centerline());
        total_length += lanelet::geometry::length(centerline_2d);
      }
      std_msgs::msg::Float64 stop_distance_msg;
      stop_distance_msg.data = total_length;
      this->stop_distance_pub->publish(stop_distance_msg);
      // RCLCPP_INFO(this->get_logger(), "Total route length: %.2f meters", total_length);
    }
    else
    {
      // RCLCPP_WARN(this->get_logger(), "Route not found between current and goal lanelet.");
    }
  }

  void MissionPlanner::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &msg)
  {
    // No need to use occupancy grid data for now
  }

  // Something should be changed here
  /**
   * @brief Callback function that processes detected clusters and marks lanelets as no-entry zones
   *
   * This function is triggered when new cluster data is received and stores cluster information for other functions. It performs the following operations:
   * 1. Finds all lanelets within 20 meters of the current vehicle position
   * 2. For each nearby lanelet, constructs a polygon representation
   * 3. For each cluster in the received message: (Remember that This method is not used for now It can be added later just in case)
   *    - Transforms cluster points from vehicle frame to global coordinates
   *    - Creates a polygon from the cluster points
   *    - Calculates the centroid of the cluster polygon
   *    - Checks if the centroid falls within any nearby lanelet
   *    - If an intersection is detected, marks the lanelet as a no-entry zone
   *
   * The function uses coordinate transformation based on the vehicle's current pose (position and yaw)
   * to convert cluster points from the vehicle's local frame to the global coordinate system.
   *
   * @param msg The incoming message containing an array of detected clusters with their bottom points
   *
   * @note Clusters with fewer than 3 points are ignored as they cannot form a valid polygon
   * @note The function avoids duplicate processing of lanelets by maintaining a set of checked IDs
   * @note Optional area filtering is available but currently commented out
   */
  void MissionPlanner::cluster_callback(const cluster_msgs::msg::ClusterPointsArray &msg)
  {
    this->bottom_points_array = msg;

    std::set<lanelet::Id> checked_lanelet_ids;
    std::vector<lanelet::ConstLanelet> lanelets_nearby;

    // Current vehicle position as a Lanelet2 point
    lanelet::BasicPoint2d current_point(this->current_position.x, this->current_position.y);

    // Find lanelets within 20 meters of the current position (avoid duplicates)
    for (const auto &lanelet : this->lanelet_map->laneletLayer)
    {
      lanelet::BasicPoint2d lanelet_center = lanelet.centerline2d().front();
      double distance = std::hypot(lanelet_center.x() - current_point.x(), lanelet_center.y() - current_point.y());
      if (distance <= 20.0 && checked_lanelet_ids.insert(lanelet.id()).second)
      {
        lanelets_nearby.push_back(lanelet);
      }
    }

    // Get the vehicle pose (position and orientation)
    double rx = this->pose.position.x;
    double ry = this->pose.position.y;
    double yaw = tf2::getYaw(this->pose.orientation);

    // Check each nearby lanelet for intersection with cluster polygons
    for (const auto &lanelet : lanelets_nearby)
    {
      // === Construct lanelet polygon ===
      BoostPolygon lanelet_polygon;
      for (const auto &pt : lanelet.leftBound())
      {
        lanelet_polygon.outer().emplace_back(pt.x(), pt.y());
      }
      for (auto it = lanelet.rightBound().begin(); it != lanelet.rightBound().end(); ++it)
      {
        lanelet_polygon.outer().emplace_back(it->x(), it->y());
      }
      lanelet_polygon.outer().push_back(lanelet_polygon.outer().front()); // Close the polygon
      boost::geometry::correct(lanelet_polygon);

      for (const auto &cluster : msg.clusters)
      {
        if (cluster.bottom_points_array.size() < 3)
          continue; // Not enough points for a polygon

        // === Construct the cluster polygon (in global frame) ===
        BoostPolygon cluster_polygon;
        for (const auto &pt : cluster.bottom_points_array)
        {
          // Transform points to global coordinates
          double gx = std::cos(yaw) * pt.x - std::sin(yaw) * pt.y + rx;
          double gy = std::sin(yaw) * pt.x + std::cos(yaw) * pt.y + ry;
          cluster_polygon.outer().emplace_back(gx, gy);
        }

        if (cluster_polygon.outer().size() < 3)
          continue;
        cluster_polygon.outer().push_back(cluster_polygon.outer().front());
        boost::geometry::correct(cluster_polygon);

        // === Calculate centroid (average point) of the cluster polygon ===
        const auto &pts = cluster_polygon.outer();
        if (pts.size() <= 1)
          continue;
        double sum_x = 0.0, sum_y = 0.0;
        int count = 0;
        // Avoid counting the closing point twice
        for (size_t i = 0; i < pts.size() - 1; ++i)
        {
          sum_x += boost::geometry::get<0>(pts[i]);
          sum_y += boost::geometry::get<1>(pts[i]);
          ++count;
        }
        if (count == 0)
          continue;
        boost::geometry::model::d2::point_xy<double> centroid(sum_x / count, sum_y / count);

        // === Check if centroid is within the lanelet polygon ===
        bool centroid_in_lanelet = boost::geometry::within(centroid, lanelet_polygon);

        // If centroid is not within the lanelet, skip this cluster
        if (!centroid_in_lanelet)
          continue;

        // === Area check (optional) ===
        double cluster_area = boost::geometry::area(cluster_polygon);
        // Uncomment if you want to filter by area:
        // if (cluster_area < 1.0) continue;

        // === Mark lanelet as no-entry due to detected obstacle ===
        if (std::find(this->no_entry_ids.begin(), this->no_entry_ids.end(), lanelet.id()) == this->no_entry_ids.end())
        {
          // RCLCPP_INFO(this->get_logger(), "No entry lanelet %ld added due to obstacle", lanelet.id());
          if (this->current_lanelet.id() == lanelet.id())
          {
            // RCLCPP_WARN(this->get_logger(), "Current lane id is the same as lanelet %ld, forcing replan", lanelet.id());
          }

          // RCLCPP_INFO(this->get_logger(), "Entering publish_no_entry_ids");
          this->publish_no_entry_ids(lanelet.id(), false);
        }
      }
    }
  }

  /**
   * @brief Publishes no-entry lanelet IDs to restrict vehicle access to specific areas
   *
   * This function manages a list of lanelet IDs that should be marked as no-entry zones.
   * It handles both station and station entry lanelets, automatically adding following
   * lanelets when a station entry is marked as no-entry.
   *
   * @param data The lanelet ID to be added to the no-entry list
   * @param from_yolo Boolean flag indicating if the request comes from YOLO detection
   *
   * @details
   * - For station entry lanelets: Adds the entry lanelet and its first following lanelet
   * - For station lanelets: Only adds the specified lanelet
   * - Validates that the lanelet exists in the map before processing
   * - Prevents duplicate entries in the no-entry list
   * - Publishes updated no-entry list via ROS2 topic for Global Planner
   * - Skips processing if lanelet is not a station/station_entry (unless from_yolo is true)
   */
  void MissionPlanner::publish_no_entry_ids(int data, bool from_yolo)
  {
    bool is_station = std::find(this->station_ids.begin(), this->station_ids.end(), data) != this->station_ids.end();
    bool is_station_entry = std::find(this->station_entry_ids.begin(), this->station_entry_ids.end(), data) != this->station_entry_ids.end();

    if (!is_station && !is_station_entry && !from_yolo)
    {
      // RCLCPP_WARN(this->get_logger(), "Lanelet %d is not a station or station entry, skipping.", data);
      return;
    }

    if (std::find(this->no_entry_ids.begin(), this->no_entry_ids.end(), data) != this->no_entry_ids.end())
    {
      // RCLCPP_INFO(this->get_logger(), "Lanelet %d is already marked as no_entry, skipping.", data);
      return;
    }

    std::vector<int32_t> new_ids;
    new_ids.push_back(data);

    auto lanelet_it = this->lanelet_map->laneletLayer.find(data);
    if (lanelet_it != this->lanelet_map->laneletLayer.end())
    {
      if (is_station_entry)
      {
        auto followings = this->routing_graph->following(*lanelet_it);
        if (!followings.empty())
        {
          int32_t lanelet_id = followings.front().id();
          if (std::find(this->no_entry_ids.begin(), this->no_entry_ids.end(), lanelet_id) == this->no_entry_ids.end() &&
              std::find(new_ids.begin(), new_ids.end(), lanelet_id) == new_ids.end())
          {
            new_ids.push_back(lanelet_id);
            RCLCPP_INFO(this->get_logger(), "Lanelet %d added to no_entry_ids due to station entry.", lanelet_id);
          }
        }
      }
      else if (is_station)
      {
        int32_t lanelet_id = lanelet_it->id();
        (void)lanelet_id;
      }
    }

    if (new_ids.empty())
    {
      RCLCPP_INFO(this->get_logger(), "No new no_entry_ids to publish.");
      return;
    }

    for (const auto &id : new_ids)
    {
      this->no_entry_ids.push_back(id);
    }

    std_msgs::msg::Int32MultiArray msg;
    msg.data = this->no_entry_ids;
    this->NoEntryPub->publish(msg);
  }

  /**
   * @brief Callback function that handles incoming goal pose messages and triggers path planning.
   *
   * This function is triggered when a new goal pose ID is received. It performs the following operations:
   * - Validates that the current position is initialized before processing
   * - Forces replanning on first-time execution or when goal pose ID changes
   * - Marks no-entry lanelets with appropriate attributes in the lanelet map
   * - Computes the shortest route from current lanelet to goal lanelet
   * - Generates centerline points for the valid path
   * - Updates internal state variables for path tracking
   *
   * @param msg ROS2 message containing the goal pose ID as an integer
   *
   * @note The function will return early if:
   *       - Current position is not initialized (is_pose_start == false)
   *       - Goal pose ID hasn't changed and no forced replan is needed
   *       - Goal lanelet ID is not found in the lanelet map
   *       - No valid route exists between current and goal lanelets
   *       - Filtered path becomes empty after removing no-entry lanelets
   *
   * @warning This function modifies the lanelet map by adding "subtype" attributes to no-entry lanelets
   */
  void MissionPlanner::goalPoseCallback(const std_msgs::msg::Int32 &msg)
  {

    if (!this->is_pose_start)
    {
      RCLCPP_WARN(this->get_logger(), "Current position not initialized, skipping goal pose callback.");
      return;
    }

    if (!this->first_time)
    {
      this->force_replan = true;
      RCLCPP_INFO(this->get_logger(), "First time goal pose received, forcing replan.");
      // Find current lane for first time initialization
    }
    if (!(this->prev_goal_pose_id == msg.data))
    {
      this->force_replan = true;
      RCLCPP_INFO(this->get_logger(), "Goal pose ID changed from %ld to %ld, forcing replan.", this->prev_goal_pose_id, msg.data);
    }
    this->prev_goal_pose_id = this->goal_pose_id;
    this->goal_pose_id = msg.data;

    if (!force_replan)
    {
      // RCLCPP_INFO(this->get_logger(), "Goal pose ID is the same as previous, no replan needed.");
      return;
    }
    force_replan = false;

    // Tüm no_entry lanelet’lere subtype ata (görsel veya başka işlem için)
    for (int32_t id : this->no_entry_ids)
    {
      auto lanelet_it = this->lanelet_map->laneletLayer.find(id);
      if (lanelet_it != this->lanelet_map->laneletLayer.end())
      {
        lanelet_it->attributes()["subtype"] = "no_entry";
      }
      // RCLCPP_INFO(this->get_logger(), "Lanelet %d marked as no_entry in goalPoseCallback", id);
    }
    this->routing_graph = lanelet::routing::RoutingGraph::build(*(this->lanelet_map), *(this->traffic_rules));

    if (this->lanelet_map->laneletLayer.find(this->goal_pose_id) == this->lanelet_map->laneletLayer.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Goal lanelet ID %ld not found in lanelet map!", this->goal_pose_id);
      this->force_replan = true;
      return;
    }

    lanelet::Lanelet goal_lanelet = this->lanelet_map->laneletLayer.get(this->goal_pose_id);

    auto route = this->routing_graph->getRoute(this->current_lanelet, goal_lanelet);
    RCLCPP_INFO(this->get_logger(), "Route found from %ld to %ld", this->current_lanelet.id(), goal_lanelet.id());
    if (!route)
    {
      RCLCPP_ERROR(this->get_logger(), "No route found from %ld to %ld", this->current_lanelet.id(), goal_lanelet.id());
      this->force_replan = true;
      return;
    }

    auto raw_path = route->shortestPath();

    std::vector<lanelet::ConstLanelet> filtered_path;
    for (const auto &lane : raw_path)
    {
      if (std::find(this->no_entry_ids.begin(), this->no_entry_ids.end(), lane.id()) == this->no_entry_ids.end())
        filtered_path.push_back(lane);
      else
        RCLCPP_WARN(this->get_logger(), "NO_ENTRY lanelet %ld path'ten filtrelendi!", lane.id());
    }

    this->shortest_path = lanelet::routing::LaneletPath(filtered_path);
    // Bilgi çıktısı
    // RCLCPP_INFO(this->get_logger(), "Shortest Path with No entry IDs:");

    for (const auto &id : this->no_entry_ids)
      RCLCPP_INFO(this->get_logger(), "No entry Lanelet ID: %d", id);

    if (this->shortest_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Shortest path is empty after routing!");
      this->force_replan = true;
      return;
    }

    this->centerline_points.clear();
    this->previous_centerpoint_idx = 0;
    this->path_ids.lanes.clear();
    for (lanelet::ConstLanelet lane : this->shortest_path)
    {
      this->path_ids.lanes.push_back(lane.id());
      for (const auto &point : lane.centerline2d())
        this->centerline_points.push_back(lanelet::BasicPoint2d(point.x(), point.y()));
    }

    RCLCPP_INFO(this->get_logger(), "Path Found ");
    this->first_time = true;

    this->previous_lane_id = this->current_lanelet.id();
  }

  bool MissionPlanner::checkAtLeastOneRouteExists()
  {
    for (const auto &station : this->lanelet_map->laneletLayer)
    {
      auto route = this->routing_graph->getRoute(this->current_lanelet, station);
      if (route)
      {
        RCLCPP_INFO(this->get_logger(), "Route exists to station with ID: %ld", station.id());
        return true;
      }
    }

    RCLCPP_WARN(this->get_logger(), "No route exists to any station.");
    return false;
  }

  void MissionPlanner::get_station_ids()
  {
    for (const auto &lanelet : this->lanelet_map->laneletLayer)
    {
      if (lanelet.hasAttribute(STATION_NAME))
      {
        this->station_ids.push_back(lanelet.id());
        RCLCPP_INFO(this->get_logger(), "Station lanelet ID: %ld", lanelet.id());
      }
    }

    for (const auto &lanelet : this->lanelet_map->laneletLayer)
    {
      if (lanelet.hasAttribute("station_entry"))
      {
        this->station_entry_ids.push_back(lanelet.id());
        RCLCPP_INFO(this->get_logger(), "Station entry lanelet ID: %ld", lanelet.id());
      }
    }
    for (const auto &lanelet : this->lanelet_map->laneletLayer)
    {
      if (lanelet.hasAttribute("parking_entry"))
      {
        RCLCPP_INFO(this->get_logger(), "Parking entry lanelet ID: %ld", lanelet.id());
        this->parking_entry_lanelet = lanelet;
      }
    }
  }

  /**
   * @brief Switches between planning modes based on obstacle detection and lane analysis.
   *
   * This method analyzes bottom point clusters from sensor data to determine whether to use
   * A* pathfinding or centerline following for navigation. The decision is made by checking
   * if obstacle centroids fall within the current lane polygon boundaries.
   *
   * Key behaviors:
   * - Initializes start time on first call for timing-based mode switching so that we can avoid unnecessary switches
   * - Constructs polygon representations of current and following lanes
   * - Transforms sensor points from sensor frame to map frame
   * - Calculates centroid of each bottom point cluster
   * - Filters clusters based on distance and relative position (ignoring close rear obstacles)
   * - Switches to centerline mode when detected obstacles are outside lane boundaries
   *   and more than 10 seconds have elapsed since the last switch
   * - Switches to A* mode immediately when an obstacle is inside a lane boundary
   * - Resets timing when switching to A* mode
   *
   * @note Requires valid clock, current_lanelet, and routing_graph to function properly
   * @note Publishes switch commands via switch_controller_pub
   * @note Uses tf_buffer_ for coordinate transformations between frames
   */
  void MissionPlanner::switchPlanningMode()
  {
    if (!clock_)
    {
      RCLCPP_WARN(get_logger(), "Clock not initialized, skipping switch planning mode");
      return;
    }

    static bool first_call = true;
    if (first_call)
    {
      start_time_ = clock_->now();
      first_call = false;
      RCLCPP_INFO(get_logger(), "start_time_ initialized on first call");
    }

    // No need to switch if in parking mode, in parking area follow centerline
    static bool parking_mode = false;
    if (this->current_lanelet == this->parking_entry_lanelet)
    {
      parking_mode = true;
    }
    if (parking_mode)
    {
      return; // No need to publish anything, we publish it in Global Planner
    }

    std::vector<lanelet::ConstLanelet> lanes{this->current_lanelet};
    if (routing_graph)
    {
      auto nexts = routing_graph->following(this->current_lanelet);
      lanes.insert(lanes.end(), nexts.begin(), nexts.end());
    }
    struct Poly
    {
      lanelet::Id id;
      std::vector<std::pair<double, double>> pts;
    };
    std::vector<Poly> polys;
    polys.reserve(lanes.size());
    for (const auto &ln : lanes)
    {
      std::vector<std::pair<double, double>> poly;
      for (const auto &pt : ln.leftBound())
        poly.emplace_back(pt.x(), pt.y());
      std::vector<std::pair<double, double>> right;
      for (const auto &pt : ln.rightBound())
        right.emplace_back(pt.x(), pt.y());
      for (int i = (int)right.size() - 1; i >= 0; --i)
        poly.push_back(right[i]);
      polys.push_back({ln.id(), std::move(poly)});
    }

    const double yaw = tf2::getYaw(pose.orientation);
    bool instant_on_trigger = false;
    bool any_evidence = false;

    for (const auto &cl : bottom_points_array.clusters)
    {
      double sx = 0, sy = 0;
      int n = 0;
      for (const auto &pt : cl.bottom_points_array)
      {
        geometry_msgs::msg::PointStamped in_pt, map_pt;
        in_pt.header.frame_id = bottom_points_array.header.frame_id;
        in_pt.point.x = pt.x;
        in_pt.point.y = pt.y;
        in_pt.point.z = pt.z;
        try
        {
          map_pt = tf_buffer_.transform(in_pt, "map");
        }
        catch (...)
        {
          continue;
        }
        sx += map_pt.point.x;
        sy += map_pt.point.y;
        ++n;
      }
      if (n == 0)
        continue;

      const double cx = sx / n, cy = sy / n;
      const double dx = cx - current_position.x;
      const double dy = cy - current_position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > max_consider_dist_)
        continue;

      double rel = std::atan2(dy, dx) - yaw;
      while (rel > M_PI)
        rel -= 2 * M_PI;
      while (rel < -M_PI)
        rel += 2 * M_PI;

      const bool is_behind = std::abs(rel) > M_PI_2;
      if (is_behind && dist < rear_ignore_dist_)
        continue;
      int astar_count = 0;
      int centerline_count = 0;

      for (const auto &pp : polys)
      {
        try
        {
          rclcpp::Time current_time = clock_->now();
          rclcpp::Duration elapsed_time = current_time - start_time_;

          // RCLCPP_INFO(get_logger(),
          //             "Cluster centroid (map): x=%.3f, y=%.3f, checking polygon %ld, time since last switch: %.3f seconds",
          //             cx, cy, pp.id, elapsed_time.seconds());

          if (!pointInPolygon(cx, cy, pp.pts) && elapsed_time.seconds() > 10.0)
          {
            astar_count++;
            // RCLCPP_INFO(get_logger(),
            //             "Cluster centroid (map): x=%.3f, y=%.3f is not in polygon %ld and counter is %d, time difference is %.3f seconds",
            //             cx, cy, pp.id, astar_count, elapsed_time.seconds());
            std_msgs::msg::Bool m;
            m.data = false;
            switch_controller_pub->publish(m);
            // The obstacle is outside the route corridor; follow the centerline.
          }
          else if (pointInPolygon(cx, cy, pp.pts))
          {
            centerline_count++;
            std_msgs::msg::Bool m;
            m.data = true;
            switch_controller_pub->publish(m);
            // The obstacle is inside the route corridor; use A* to avoid it.

            start_time_ = current_time;
            // RCLCPP_INFO(get_logger(), "A* started at time: %.3f", start_time_.seconds());
            return;
          }
        }
        catch (const std::runtime_error &e)
        {
          // RCLCPP_ERROR(get_logger(), "Time calculation error in switchPlanningMode: %s", e.what());
          start_time_ = clock_->now();
          continue;
        }
      }
    }
  }

  /**
   * @brief Checks for pedestrians on the current and upcoming lane paths and publishes stop signal if detected.
   *
   * This method performs pedestrian detection by:
   * 1. Collecting candidate lanes (current lane and following lanes from routing graph)
   * 2. Converting lane boundaries to polygons for geometric collision detection
   * 3. Processing LiDAR cluster data to calculate pedestrian centroids in map frame
   * 4. Performing point-in-polygon tests to determine if pedestrians are within lane boundaries
   * 5. Publishing a boolean stop message based on detection results
   *
   * The function transforms LiDAR points from sensor frame to map frame using TF2,
   * calculates cluster centroids, and tests if any centroid falls within the polygonal
   * representation of current or upcoming lanes. If a pedestrian is detected in any
   * candidate lane, a stop signal is published.
   *
   * @note Requires valid TF transforms between LiDAR frame and map frame
   * @note Uses the pointInPolygon utility function for geometric intersection testing
   * @note Publishes to stop_pub topic with std_msgs::msg::Bool message type
   */
  void MissionPlanner::checkPedestrianOnPath()
  {
    std::vector<lanelet::ConstLanelet> candidateLanes;
    candidateLanes.push_back(this->current_lanelet);
    if (this->routing_graph)
    {
      auto nexts = this->routing_graph->following(this->current_lanelet);
      candidateLanes.insert(candidateLanes.end(), nexts.begin(), nexts.end());
    }

    std::vector<std::vector<std::pair<double, double>>> candidatePolys;
    for (const auto &lane : candidateLanes)
    {
      std::vector<std::pair<double, double>> poly;

      for (const auto &pt : lane.leftBound())
      {
        poly.emplace_back(pt.x(), pt.y());
      }

      auto rightBound = lane.rightBound();
      std::vector<std::pair<double, double>> right_points;
      for (const auto &pt : rightBound)
      {
        right_points.emplace_back(pt.x(), pt.y());
      }

      for (int i = right_points.size() - 1; i >= 0; --i)
      {
        poly.push_back(right_points[i]);
      }

      candidatePolys.push_back(poly);
    }

    bool pedestrian_detected = false;

    for (const auto &cluster : this->bottom_points_array.clusters)
    {
      int num = 0;
      double total_x = 0.0, total_y = 0.0;

      // Calculate cluster centroid in map frame
      for (const auto &point : cluster.bottom_points_array)
      {
        geometry_msgs::msg::PointStamped in_pt;
        in_pt.header.frame_id = this->bottom_points_array.header.frame_id; // LiDAR frame
        in_pt.point.x = point.x;
        in_pt.point.y = point.y;
        in_pt.point.z = point.z;

        geometry_msgs::msg::PointStamped map_pt;
        try
        {
          map_pt = tf_buffer_.transform(in_pt, "map");
          total_x += map_pt.point.x;
          total_y += map_pt.point.y;
          ++num;
        }
        catch (const tf2::TransformException &)
        {
          continue;
        }
      }

      if (num == 0)
      {
        // Nothing transformable in this cluster
        continue;
      }

      const double cx = total_x / static_cast<double>(num);
      const double cy = total_y / static_cast<double>(num);

      for (size_t i = 0; i < candidatePolys.size(); ++i)
      {
        const auto &poly = candidatePolys[i];
        if (pointInPolygon(cx, cy, poly))
        {
          pedestrian_detected = true;
          // RCLCPP_INFO(this->get_logger(), "Pedestrian detected in lane %ld at (%.3f, %.3f)",
          //             candidateLanes[i].id(), cx, cy);
          break; // Found in this lane, no need to check other lanes for this cluster
        }
      }
    }

    // --- Final result ---
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = pedestrian_detected;
    this->stop_pub->publish(stop_msg);
  }

  /**
   * @brief Determines if a point is inside a polygon using the ray casting algorithm.
   *
   * This function implements the ray casting algorithm to determine if a given point lies inside a polygon.
   * The algorithm works by casting a ray from the point to infinity and counting how many times it intersects with the
   * polygon's edges. If the number of intersections is odd, the point is inside; if even,
   * the point is outside.
   *
   * @param x The x-coordinate of the point to test
   * @param y The y-coordinate of the point to test
   * @param poly A vector of pairs representing the vertices of the polygon in order.
   *             Each pair contains (x, y) coordinates of a vertex.
   *
   * @return true if the point (x, y) is inside the polygon, false otherwise
   *
   * @note The polygon is assumed to be a simple polygon (no self-intersections)
   * @note The vertices in the polygon should be ordered (either clockwise or counterclockwise)
   * @note Points exactly on the polygon boundary may have undefined behavior depending on
   *       floating-point precision
   */
  bool MissionPlanner::pointInPolygon(double x, double y, const std::vector<std::pair<double, double>> &poly)
  {
    // RCLCPP_INFO(this->get_logger(), "Checking point (%.3f, %.3f) against polygon with %zu vertices", x, y, poly.size());

    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
    {
      const auto &[xi, yi] = poly[i];
      const auto &[xj, yj] = poly[j];

      // RCLCPP_INFO(this->get_logger(), "Edge %zu: (%.3f, %.3f) -> (%.3f, %.3f)", i, xi, yi, xj, yj);

      if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
      {
        inside = !inside;
        // RCLCPP_INFO(this->get_logger(), "Ray intersection found at edge %zu, inside status: %s", i, inside ? "true" : "false");
      }
    }

    // RCLCPP_INFO(this->get_logger(), "Point (%.3f, %.3f) is %s the polygon", x, y, inside ? "inside" : "outside");
    return inside;
  }

  /**
   * @brief Callback function for processing YOLO object detection results
   *
   * This function is called when a new YOLO detection message is received.
   * It processes the incoming detection array containing objects detected by
   * the camera-based YOLO neural network. Then it performs the necessary
   * actions based on the detected traffic signs.
   *
   * @param msg The detection array message containing detected objects with
   *            their bounding boxes, confidence scores, and class labels
   */

  // yol kapatmada bazen çalışmıyor bunun için araştırmalar yap.
  void MissionPlanner::yoloCallback(const sac_interfaces::msg::CameraDetectionArray &msg)
  {
    const double observation_age = (get_clock()->now() - rclcpp::Time(msg.header.stamp)).seconds();
    // Bag /clock updates can lag a camera stamp by a few milliseconds.
    // Match the camera-policy tolerance while still rejecting stale detections.
    if ((this->get_parameter("enforce_freshness").as_bool() &&
         (observation_age < -0.1 || observation_age > 0.75)) || !this->routing_graph ||
        this->current_lanelet.id() == 0) {
      return;
    }
    if (msg.detections.empty())
    {
      // RCLCPP_INFO(this->get_logger(), "No traffic signs detected.");
      return;
    }

    lanelet::Id straight_id = 0;
    lanelet::Id left_id = 0;
    lanelet::Id right_id = 0;

    std::vector<lanelet::routing::LaneletRelation> relations =
        this->routing_graph->followingRelations(this->current_lanelet);

    for (const auto &rel : relations)
    {
      if (rel.relationType != lanelet::routing::RelationType::Successor)
      {
        continue;
      }
      // Class-specific route restrictions are applied below. A sign's distance
      // alone is not evidence that every successor lane is blocked.
      // turn_direction kontrolü
      if (rel.lanelet.hasAttribute("turn_direction"))
      {
        auto dir = rel.lanelet.attribute("turn_direction").value();

        if (dir == "straight")
        {
          straight_id = rel.lanelet.id();
        }
        else if (dir == "left")
        {
          left_id = rel.lanelet.id();
        }
        else if (dir == "right")
        {
          right_id = rel.lanelet.id();
        }
      }
    }

    int counter = 0;

    for (const auto &traffic_sign : msg.detections)
    {
      // Unknown (-1), NaN and stale metric distances cannot authorize behavior.
      if (!std::isfinite(traffic_sign.distance) || traffic_sign.distance <= 0.0 ||
          traffic_sign.distance > 10.0)
      {
        continue;
      }

      switch (traffic_sign.label)
      {
      case sac_interfaces::msg::CameraDetection::NO_ENTRY:
      {
        if (straight_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(straight_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_straight = *it;
            lane_straight.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(straight_id, true);
            this->force_replan = true;
          }
        }
        break;
      case sac_interfaces::msg::CameraDetection::PEDESTRIANS_CROSSING:
      {
        if (traffic_sign.distance < 5.0)
        {
          this->checkPedestrianOnPath();
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::ONE_WAY_STRAIGHT:
      {
        if (left_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(left_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_left = *it;
            lane_left.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(left_id, true);
            this->force_replan = true;
          }
        }
        if (right_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(right_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_right = *it;
            lane_right.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(right_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::GO_STRAIGHT_OR_TURN_LEFT:
      {
        if (right_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(right_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_right = *it;
            lane_right.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(right_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::GO_STRAIGHT_OR_TURN_RIGHT:
      {
        if (left_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(left_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_left = *it;
            lane_left.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(left_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::TURN_RIGHT_AHEAD:
      {
        if (straight_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(straight_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_straight = *it;
            lane_straight.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(straight_id, true);
            this->force_replan = true;
          }
        }
        if (left_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(left_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_left = *it;
            lane_left.attributes()["subtype"] = std::string("no_entry");
            this->no_entry_ids.push_back(left_id);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::TURN_LEFT_AHEAD:
      {
        if (straight_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(straight_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_straight = *it;
            lane_straight.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(straight_id, true);
            this->force_replan = true;
          }
        }
        if (right_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(right_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_right = *it;
            lane_right.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(right_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::TURN_LEFT:
      {
        if (right_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(right_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_right = *it;
            lane_right.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(right_id, true);
            this->force_replan = true;
          }
        }
        if (straight_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(straight_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_straight = *it;
            lane_straight.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(straight_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::TURN_RIGHT:
      {
        if (left_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(left_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_left = *it;
            lane_left.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(left_id, true);
            this->force_replan = true;
          }
        }
        if (straight_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(straight_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_straight = *it;
            lane_straight.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(straight_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::NO_RIGHT_TURN:
      {
        if (right_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(right_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_right = *it;
            lane_right.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(right_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::NO_LEFT_TURN:
      {
        if (left_id != 0)
        {
          auto it = this->lanelet_map->laneletLayer.find(left_id);
          if (it != this->lanelet_map->laneletLayer.end())
          {
            lanelet::Lanelet lane_left = *it;
            lane_left.attributes()["subtype"] = std::string("no_entry");
            this->publish_no_entry_ids(left_id, true);
            this->force_replan = true;
          }
        }
      }
      break;
      case sac_interfaces::msg::CameraDetection::RED_LIGHT:
      { // if (!map_.laneletLayer[laneletID_].attributes.contains("light_detection"))
        //     continue;
        // traffic_light_pub_->publish(std_msgs::msg::Bool().set__data(true));
        // RCLCPP_INFO(this->get_logger(), "Red light detected on lane ");
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        this->stop_pub->publish(stop_msg);
      }
      break;
      case sac_interfaces::msg::CameraDetection::GREEN_LIGHT:
      {
        // RCLCPP_INFO(this->get_logger(), "Green light detected on lane ");
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = false;
        this->stop_pub->publish(stop_msg);
      }
      break;
      case sac_interfaces::msg::CameraDetection::STOP:
      {
        if (this->get_parameter("enable_stop_sign").as_bool() &&
            traffic_sign.distance < 8.0)
        {
          this->checkPedestrianOnPath();
        }
      }
      break;
      default:
        break;
      }
        ++counter;
      }

      this->load_the_graph();
    }
  }

  void MissionPlanner::telemetryCallback(const sac_interfaces::msg::Telemetry &msg)
  {
    this->vehicle_speed_ptr = std::make_shared<double>(static_cast<double>(msg.motor_velocity)); // rpm/s
    this->vehicle_speed_update_time = this->get_clock()->now();                                  // Update the last speed update time
  }

  double MissionPlanner::getLaneletYaw(const lanelet::ConstLanelet &lanelet)
  {
    const auto &centerline = lanelet.centerline();
    if (centerline.size() < 2)
      return 0.0;
    auto p1 = centerline[centerline.size() - 2];
    auto p2 = centerline[centerline.size() - 1];
    return std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
  }

  void MissionPlanner::readLaneletMap()
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

  void MissionPlanner::load_the_graph()
  {
    this->routing_graph = lanelet::routing::RoutingGraph::build(*(this->lanelet_map), *(this->traffic_rules));
  }

} // end of namespace smart_car

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::MissionPlanner)
