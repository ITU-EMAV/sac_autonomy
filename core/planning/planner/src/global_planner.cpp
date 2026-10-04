#include "global_planner.hpp"

// TODO: Add stopping for stop signs and detect occupied parking lots

namespace smart_car
{
  GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions &options)
      : Node("global_planner_exe", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_), at_goal_station(false)
  {
    RCLCPP_INFO(this->get_logger(), "global_planner is initialized!");

    this->declare_parameter<double>("origin_pose._x", 0.0);
    this->declare_parameter<double>("origin_pose._y", 0.0);
    this->declare_parameter<std::string>("osm_path", "./map.osm");
    this->declare_parameter<std::string>("station_name", "station");
    this->declare_parameter<std::string>("park_name", "park");
    this->declare_parameter<bool>("rviz_pose", false);

    this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
    this->declare_parameter<std::string>("rviz_goal_topic", "rviz_goal_pose");
    this->declare_parameter<std::string>("pose_topic", "pose");
    this->declare_parameter<std::string>("cluster_topic", "bottom_points");
    this->declare_parameter<lanelet::Id>("first_goal_id", 0);
    this->declare_parameter<lanelet::Id>("station_goal_lanelet_id", 0);
    this->declare_parameter<lanelet::Id>("parking_lot_entry_id", 0);
    this->declare_parameter<bool>("is_first_day", false);
    this->declare_parameter<double>("stopping_at_station_tolerance", 3.0);
    this->declare_parameter<double>("stopping_at_park_tolerance", 3.0);
    this->declare_parameter<std::string>("yolo_topic", " ");
    this->declare_parameter<std::string>("no_entry_topic", " ");
    this->declare_parameter<std::string>("switch_to_centerline_topic", " ");
    this->declare_parameter<std::string>("speed_limit_topic", "/speed_limit");
    this->declare_parameter<std::string>("station_marker_topic", "/station_marker");

    this->origin_x = this->get_parameter("origin_pose._x").as_double();
    this->origin_y = this->get_parameter("origin_pose._y").as_double();
    this->osm_path = this->get_parameter("osm_path").as_string();
    this->station_name = this->get_parameter("station_name").as_string();
    this->park_name = this->get_parameter("park_name").as_string();
    this->rviz_pose = this->get_parameter("rviz_pose").as_bool();

    std::string RVIZ_GOAL_TOPIC = this->get_parameter("rviz_goal_topic").as_string();
    std::string GOAL_POSE_TOPIC = this->get_parameter("goal_pose_topic").as_string();
    std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
    std::string CLUSTER_TOPIC = this->get_parameter("cluster_topic").as_string();

    this->first_goal_id = this->get_parameter("first_goal_id").as_int();
    this->station_goal_lanelet_id = this->get_parameter("station_goal_lanelet_id").as_int();
    this->parking_lot_entry_id = this->get_parameter("parking_lot_entry_id").as_int();
    this->is_first_day = this->get_parameter("is_first_day").as_bool();
    this->stopping_at_station_tolerance = this->get_parameter("stopping_at_station_tolerance").as_double();
    this->stopping_at_park_tolerance = this->get_parameter("stopping_at_park_tolerance").as_double();

    std::string YOLO_TOPIC = this->get_parameter("yolo_topic").as_string();
    std::string NO_ENTRY_TOPIC = this->get_parameter("no_entry_topic").as_string();
    std::string SWITCH_TO_CENTERLINE_TOPIC = this->get_parameter("switch_to_centerline_topic").as_string();
    std::string SPEED_LIMIT_TOPIC = this->get_parameter("speed_limit_topic").as_string();
    std::string STATION_MARKER_TOPIC = this->get_parameter("station_marker_topic").as_string();

    // Print all parameters
    RCLCPP_INFO(this->get_logger(), "===== Global Planner Parameters =====");
    RCLCPP_INFO(this->get_logger(), "origin_pose._x: %.2f", this->origin_x);
    RCLCPP_INFO(this->get_logger(), "origin_pose._y: %.2f", this->origin_y);
    RCLCPP_INFO(this->get_logger(), "osm_path: %s", this->osm_path.c_str());
    RCLCPP_INFO(this->get_logger(), "station_name: %s", this->station_name.c_str());
    RCLCPP_INFO(this->get_logger(), "park_name: %s", this->park_name.c_str());
    RCLCPP_INFO(this->get_logger(), "rviz_pose: %s", this->rviz_pose ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: %s", GOAL_POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "rviz_goal_topic: %s", RVIZ_GOAL_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "pose_topic: %s", POSE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "cluster_topic: %s", CLUSTER_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "first_goal_id: %ld", this->first_goal_id);
    RCLCPP_INFO(this->get_logger(), "parking_lot_entry_id: %ld", this->parking_lot_entry_id);
    RCLCPP_INFO(this->get_logger(), "is_first_day: %s", this->is_first_day ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "stopping_at_station_tolerance: %.2f", this->stopping_at_station_tolerance);
    RCLCPP_INFO(this->get_logger(), "stopping_at_park_tolerance: %.2f", this->stopping_at_park_tolerance);
    RCLCPP_INFO(this->get_logger(), "yolo_topic: %s", YOLO_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "no_entry_topic: %s", NO_ENTRY_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "switch_to_centerline_topic: %s", SWITCH_TO_CENTERLINE_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "speed_limit_topic: %s", SPEED_LIMIT_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "station_marker_topic: %s", STATION_MARKER_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "======================================");

    this->readLaneletMap();
    this->readIds();

    this->goal_pub = this->create_publisher<std_msgs::msg::Int32>(GOAL_POSE_TOPIC, 10);
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(POSE_TOPIC, rclcpp::QoS(1), std::bind(&GlobalPlanner::pose_callback, this, std::placeholders::_1));
    this->speed_limit_pub = this->create_publisher<std_msgs::msg::Float64>(SPEED_LIMIT_TOPIC, 10);
    this->no_entry_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(NO_ENTRY_TOPIC, 10, std::bind(&GlobalPlanner::no_entry_arr_sub, this, std::placeholders::_1));
    this->yolo_sub = this->create_subscription<sac_interfaces::msg::CameraDetectionArray>(YOLO_TOPIC, rclcpp::QoS(1), std::bind(&GlobalPlanner::yoloCallback, this, std::placeholders::_1));
    this->switch_to_centerline_pub = this->create_publisher<std_msgs::msg::Bool>(SWITCH_TO_CENTERLINE_TOPIC, 10);
    this->station_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(STATION_MARKER_TOPIC, 10);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.transient_local().transient_local().reliable();
    this->stop_pub = this->create_publisher<std_msgs::msg::Bool>("/stop", qos_profile);

    if (this->rviz_pose)
    {
      this->rviz_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(RVIZ_GOAL_TOPIC, 10, std::bind(&GlobalPlanner::rviz_goal_pose_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "RVIZ goal pose is enabled");
    }

    this->cluster_sub = this->create_subscription<cluster_msgs::msg::ClusterPointsArray>(CLUSTER_TOPIC, rclcpp::QoS(1), std::bind(&GlobalPlanner::cluster_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "RVIZ goal pose %s; configured station goal lanelet ID: %ld",
                this->rviz_pose ? "enabled" : "disabled", this->station_goal_lanelet_id);

    station1.pose.position.x = 196.41282653808594;
    station1.pose.position.y = 181.0134735107422;
    station1.pose.position.z = 0.0;

    station2.pose.position.x = 201.596923828125;
    station2.pose.position.y = 113.39493560791016;
    station2.pose.position.z = 0.0;

    station3.pose.position.x = 148.4650421142578;
    station3.pose.position.y = 137.6022186279297;
    station3.pose.position.z = 0.0;

    station4.pose.position.x = 277.93658447265625;
    station4.pose.position.y = 371.33294677734375;
    station4.pose.position.z = 0.0;

    // Bind the station target to the selected Lanelet map instead of using a
    // hard-coded map coordinate that may lie far outside the active map.
    if (this->station_goal_lanelet_id > 0)
    {
      auto goal_it = this->lanelet_map->laneletLayer.find(this->station_goal_lanelet_id);
      if (goal_it == this->lanelet_map->laneletLayer.end())
      {
        throw std::invalid_argument("station_goal_lanelet_id is not present in the selected map");
      }
      const auto centerline = goal_it->centerline2d();
      if (centerline.empty())
      {
        throw std::invalid_argument("station_goal_lanelet_id has no centerline");
      }
      station4.pose.position.x = centerline.back().x();
      station4.pose.position.y = centerline.back().y();
      RCLCPP_INFO(this->get_logger(), "Station goal: lanelet %ld endpoint (%.2f, %.2f)",
                  this->station_goal_lanelet_id, station4.pose.position.x,
                  station4.pose.position.y);
    }

    // Order of stations to visit
    this->station_positions.push_back(station4);
    // this->station_positions.push_back(station1);
    // this->station_positions.push_back(station3);
    // this->station_positions.push_back(station2);

    clock_ = this->get_clock();
  }

  void GlobalPlanner::readLaneletMap()
  {
    lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});
    lanelet::projection::UtmProjector projector(origin);
    this->lanelet_map = lanelet::load(this->osm_path, projector);

    if (!this->lanelet_map)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");

      auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
          lanelet::Locations::Germany, lanelet::Participants::Vehicle);

      auto cost = std::make_shared<lanelet::routing::RoutingCostDistance>(0.0); // sadece uzunluk
      lanelet::routing::RoutingCostPtrs costPtrs{cost};

      this->routing_graph = lanelet::routing::RoutingGraph::build(
          *lanelet_map,
          *traffic_rules,
          costPtrs);
    }
  }

  // We do not use it because we are required to reach lat lon positions instead of lanelet IDs. Goal position may be any lane.
  // In case of using lanelet IDs as goals, this function can be used to read station and park ids from the map.
  void GlobalPlanner::readIds() // read station and park ids
  {
    // for (const auto &lanelet : this->lanelet_map->laneletLayer)
    // {
    //   if (lanelet.hasAttribute(this->station_name))
    //   {
    //     this->station_ids.push_back(lanelet.id());
    //   }
    //   else if (lanelet.hasAttribute(this->park_name))
    //   {
    //     this->park_ids.push_back(lanelet.id());
    //   }
    // }
  }

  void GlobalPlanner::rviz_goal_pose_callback(const geometry_msgs::msg::PoseStamped &msg)
  {
    if (!this->rviz_pose)
    {
      RCLCPP_WARN(this->get_logger(), "RVIZ goal pose is not enabled!");
      return;
    }

    const lanelet::BasicPoint2d goal_point(msg.pose.position.x, msg.pose.position.y);

    RCLCPP_INFO(this->get_logger(), "Goal pose received: x: %f, y: %f", goal_point.x(), goal_point.y());

    std_msgs::msg::Int32 goal_id_msg;
    goal_id_msg.data = this->lanelet_map->laneletLayer.nearest(goal_point, 1)[0].id();

    this->goal_pub->publish(goal_id_msg);
    this->current_goal_station_id = goal_id_msg.data;
  }

  void GlobalPlanner::yoloCallback(const sac_interfaces::msg::CameraDetectionArray &msg)
  {
    if (!this->do_park)
    {
      return;
    }
    static bool first_call = false;
    static bool last_mission_going_to_park = false;
    if (last_mission_going_to_park)
    {
      // RCLCPP_INFO(this->get_logger(), "No need to use yolocallback, parking lot has been decided.");
      return;
    }
    if (!first_call)
    {
      this->thinking_free_park_time_ = this->clock_->now();
      first_call = true;
      // RCLCPP_INFO(this->get_logger(), "thinking_free_park_time_ initialized on first call");
    }
    rclcpp::Time current_time = clock_->now();
    rclcpp::Duration elapsed_time = current_time - this->thinking_free_park_time_;
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, // Log every 5 seconds
    //                      "Elapsed time since starting to think about parking: %.2f seconds",
    //                      elapsed_time.seconds());
    if (elapsed_time > rclcpp::Duration::from_seconds(30.0))
    {
      // RCLCPP_INFO(this->get_logger(), "Thinking time exceeded 30 seconds, going to first parking lot.");
      this->do_park = false; // Stop trying to park
      std_msgs::msg::Int32 goal_id_msg;
      goal_id_msg.data = PARK_LANES[0];
      this->goal_pub->publish(goal_id_msg);
      this->current_goal_station_id = PARK_LANES[0];
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = false;
      this->stop_pub->publish(stop_msg);
      last_mission_going_to_park = true;
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "YOLO callback triggered with %zu detections", msg.detections.size());
    std::vector<std::pair<int, double>> parking_data;
    int counter = 0;
    for (const auto &detection : msg.detections)
    {
      if (detection.confidence > 0.65)
      {
        if (counter < 9)
        {
          switch (detection.label)
          {
          case sac_interfaces::msg::CameraDetection::PARKING:
          {
            RCLCPP_INFO(this->get_logger(), "Parking detected with ID: %d", PARK_LANES[counter]);
            this->current_goal_station_id = PARK_LANES[counter];
            std_msgs::msg::Int32 goal_id_msg;
            int current_park_id = PARK_LANES[counter];
            if (std::find(this->closed_park_ids.begin(), this->closed_park_ids.end(), current_park_id) == this->closed_park_ids.end())
            {
              goal_id_msg.data = PARK_LANES[counter];
              this->goal_pub->publish(goal_id_msg);
              this->do_park = false; // Stop going to parking lot
              std_msgs::msg::Bool switch_msg;
              switch_msg.data = true; // Switch to centerline mode
              this->switch_to_centerline_pub->publish(switch_msg);
              RCLCPP_INFO(this->get_logger(), "Switching to centerline mode after parking detection.");
              std_msgs::msg::Bool stop_msg;
              stop_msg.data = false;
              this->stop_pub->publish(stop_msg);
              last_mission_going_to_park = true;
            }
          }
          break;
          case sac_interfaces::msg::CameraDetection::NO_PARKING:
            RCLCPP_INFO(this->get_logger(), "No Parking detected with ID: %d", PARK_LANES[counter]);
            break;
          case sac_interfaces::msg::CameraDetection::DISABLED_PARKING:
            RCLCPP_INFO(this->get_logger(), "Disabled Parking detected with ID: %d", PARK_LANES[counter]);
            break;
          default:
          {
            RCLCPP_INFO(this->get_logger(), "No valid parking detected trying again.");
            std_msgs::msg::Bool stop_msg;
            stop_msg.data = true;
            this->stop_pub->publish(stop_msg);
          }
          break;
          }
        }
      }
      counter++;
    }
  }

  void GlobalPlanner::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    this->current_position.x = msg.pose.pose.position.x;
    this->current_position.y = msg.pose.pose.position.y;

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

    lanelet::BasicPoint2d current_point(this->current_position.x, this->current_position.y);
    auto current_lane = this->lanelet_map->laneletLayer.nearest(current_point, 1)[0];
    static bool parking_entry = false;
    static bool stop_once = false;
    if (parking_entry)
    {
      // Mesafe toleransı (metre)
      constexpr double kParkingReachTol = 3.0;

      // Hedef parking lot entry lanelet'in merkez noktasını al
      auto target_pt = this->lanelet_map->laneletLayer.get(this->current_goal_station_id).centerline2d().back();

      double dx = this->current_position.x - target_pt.x();
      double dy = this->current_position.y - target_pt.y();
      double dist_to_parking_entry = std::hypot(dx, dy);
      // RCLCPP_INFO(this->get_logger(), "Distance to parking entry lanelet: %.2f meters", dist_to_parking_entry);

      if (dist_to_parking_entry <= kParkingReachTol)
      {
        // RCLCPP_INFO(this->get_logger(),
        //             "Reached parking (%.2f m away). Stopping vehicle.",
        //             dist_to_parking_entry);

        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        this->stop_pub->publish(stop_msg);
      }
      return;
    }

    if (current_lane.id() == this->parking_lot_entry_id)
    {
      if (!stop_once) // stop for detecting parking signs properly.
      {
        parking_entry = true;
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        this->stop_pub->publish(stop_msg);

        this->wait_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), // Fixed: was 500 seconds
            [this]()
            {
              this->wait_timer_->cancel(); // Stop the timer after execution
              std_msgs::msg::Bool stop_msg;
              stop_msg.data = false;
              this->stop_pub->publish(stop_msg);
              stop_once = true;
            });
      }
    }

    if (this->is_first_day)
    {
      if (current_lane.hasAttribute("speed_limit"))
      {
        auto a = current_lane.attributes()["speed_limit"].asDouble();


        // We dont use this one 
        if (a.has_value())
        {
          // Set speed limit based on parking mode
          // Legacy /speed_limit carries SI m/s policy values. The Lanelet
          // attribute only gates publication; its raw units are not used.
          double speed_limit_mps = this->do_park ? 2.0 : 4.0;
          std_msgs::msg::Float64 speed_limit_msg;
          speed_limit_msg.data = speed_limit_mps;
          this->speed_limit_pub->publish(speed_limit_msg);
        }

        // Static variable to track parking entry state
        static bool going_to_parking_entry = false;
        static bool is_reached_parking_entry = (current_lane.id() == this->parking_lot_entry_id);

        // If no stations left → go to parking lot entry lanelet
        if (this->station_positions.empty())
        {
          if (!going_to_parking_entry)
          {
            try
            {
              // Publish goal to motion planner
              if (!parking_entry)
              {
                std_msgs::msg::Int32 goal_id_msg;
                goal_id_msg.data = this->parking_lot_entry_id;
                this->goal_pub->publish(goal_id_msg);

                this->current_goal_station_id = this->parking_lot_entry_id;
                going_to_parking_entry = true;
                // RCLCPP_INFO(this->get_logger(), "Publishing goal to parking lot entry ID %ld", this->parking_lot_entry_id);
              }

              // RCLCPP_INFO(this->get_logger(), "No stations left. Going to parking lot entry lanelet %ld", this->parking_lot_entry_id);
            }
            catch (const std::exception &e)
            {
              // RCLCPP_ERROR(this->get_logger(), "Error finding parking lot entry lanelet: %s", e.what());
            }
          }
          else
          {
            if (current_lane.id() == this->parking_lot_entry_id)
            {
              // RCLCPP_INFO(this->get_logger(), "Reached parking lot entry. Initiating parking sequence.");
              this->goto_park();
            }
          }
          return;
        }

        // Process current station goal
        const auto &goal = this->station_positions.front();
        { // Publish station marker for visualization
          visualization_msgs::msg::Marker station_marker;
          station_marker.header.frame_id = "map";
          station_marker.header.stamp = this->clock_->now();
          station_marker.id = 0;
          station_marker.type = visualization_msgs::msg::Marker::SPHERE;
          station_marker.action = visualization_msgs::msg::Marker::ADD;
          station_marker.pose = goal.pose;
          station_marker.pose.position.z += 1.0; // Raise marker above ground for visibility
          station_marker.scale.x = 2.0;
          station_marker.scale.y = 2.0;
          station_marker.scale.z = 2.0;
          station_marker.color.r = 1.0;
          station_marker.color.g = 0.0;
          station_marker.color.b = 0.0;
          station_marker.color.a = 1.0;
          this->station_marker_pub->publish(station_marker);
        }
        // RCLCPP_INFO(this->get_logger(),
        //             "Current goal station: (%.2f, %.2f)",
        //             goal.pose.position.x, goal.pose.position.y);

        // Find the lanelet containing the goal station
        lanelet::BasicPoint2d goal_point(goal.pose.position.x, goal.pose.position.y);
        auto nearest_lanelets = this->lanelet_map->laneletLayer.nearest(goal_point, 1);

        if (!nearest_lanelets.empty()) // if it is available in the map, publish
        {
          this->current_goal_station_id = this->station_goal_lanelet_id > 0 ?
              this->station_goal_lanelet_id : nearest_lanelets[0].id();

          // Publish the goal lanelet ID
          std_msgs::msg::Int32 goal_id_msg;
          goal_id_msg.data = this->current_goal_station_id;
          this->goal_pub->publish(goal_id_msg);
        }

        // Calculate distance to goal
        const double dist = std::hypot(this->current_position.x - goal.pose.position.x, this->current_position.y - goal.pose.position.y);

        if (dist <= this->stopping_at_station_tolerance)
        {
          this->station_positions.erase(this->station_positions.begin());

          // Wait for 10 seconds so that the vehicle is not stuck
          rclcpp::Time current_time = clock_->now();
          rclcpp::Duration elapsed_time = current_time - start_time_;

          if (elapsed_time < rclcpp::Duration::from_seconds(10.0)) // station wait time + 10
          {
            // RCLCPP_INFO(this->get_logger(),
            //             "Elapsed time since last stop: %.2f seconds. Waiting for 10 seconds total.",
            //             elapsed_time.seconds());
            return;
          }

          std_msgs::msg::Bool stop_msg;
          stop_msg.data = true;
          this->stop_pub->publish(stop_msg);

          // Wait for 5 seconds
          this->wait_timer_ = this->create_wall_timer(
              std::chrono::seconds(5),
              [this]()
              {
                this->wait_timer_->cancel(); // Stop the timer after execution
                // RCLCPP_INFO(this->get_logger(), "5-second wait completed, resuming operation.");

                // Resume vehicle operation
                std_msgs::msg::Bool stop_msg;
                stop_msg.data = false;
                this->stop_pub->publish(stop_msg);
                this->start_time_ = this->clock_->now(); // update

                // Publish goal to next station if stations remain
                if (!this->station_positions.empty())
                {
                  const auto &next_goal = this->station_positions.front();
                  lanelet::BasicPoint2d goal_point(next_goal.pose.position.x, next_goal.pose.position.y);
                  auto nearest_lanelets = this->lanelet_map->laneletLayer.nearest(goal_point, 1);

                  if (!nearest_lanelets.empty())
                  {
                    auto goal_lanelet = nearest_lanelets[0];
                    this->current_goal_station_id = goal_lanelet.id();

                    std_msgs::msg::Int32 goal_id_msg;
                    goal_id_msg.data = this->current_goal_station_id;
                    this->goal_pub->publish(goal_id_msg);
                  }
                }

                // Log next station if available
                if (!this->station_positions.empty())
                {
                  const auto &next_goal = this->station_positions.front();
                  RCLCPP_INFO(this->get_logger(), "Next station: (%.2f, %.2f)",
                              next_goal.pose.position.x, next_goal.pose.position.y);
                }
                else
                {
                  RCLCPP_INFO(this->get_logger(), "All stations completed. Will head to parking on next iteration.");
                  this->current_goal_station_id = this->parking_lot_entry_id;
                }
              });
        }
        else
        {
          // RCLCPP_INFO(this->get_logger(),
          //             "Heading to station (%.2f, %.2f), dist=%.2f",
          //             goal.pose.position.x, goal.pose.position.y, dist);
        }
      }
      return;
    }

    if (this->rviz_pose)
    {
      static bool parked = false;

      if (current_lane.hasAttribute("speed_limit"))
      {
        auto a = current_lane.attributes()["speed_limit"].asDouble();
        if (a.has_value())
        {
          if (this->do_park)
          {
            double speed_limit_mps = 2.0;
            std_msgs::msg::Float64 speed_limit_msg;
            speed_limit_msg.data = speed_limit_mps;
            this->speed_limit_pub->publish(speed_limit_msg);
          }
          else
          {
            double speed_limit_mps = 4.0;
            std_msgs::msg::Float64 speed_limit_msg;
            speed_limit_msg.data = speed_limit_mps;
            this->speed_limit_pub->publish(speed_limit_msg);
          }
        }
        RCLCPP_INFO(this->get_logger(), "Current lane id %ld Goal ID: %ld", current_lane.id(), this->current_goal_station_id);
        if (this->is_finished && current_lane.id() == this->current_goal_station_id)
        {
          // RCLCPP_INFO(this->get_logger(), "Finished.");
          std_msgs::msg::Bool stop_msg;
          stop_msg.data = true;
          this->stop_pub->publish(stop_msg);
        }
        if (current_lane.id() == this->current_goal_station_id && !parked)
        {
          RCLCPP_INFO(this->get_logger(), "Already at the goal station, no further action required.");
          std_msgs::msg::Bool stop_msg;
          stop_msg.data = true;
          this->stop_pub->publish(stop_msg);
          if (!parked)
          {
            RCLCPP_INFO(this->get_logger(), "Parking at the goal station: %ld", this->current_goal_station_id);
            parked = true;
            this->wait_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]()
                {
                  this->wait_timer_->cancel(); // Stop the timer after execution
                  RCLCPP_INFO(this->get_logger(), "5-second timer completed, resuming operation.");
                  std_msgs::msg::Bool stop_msg;
                  stop_msg.data = false;
                  this->stop_pub->publish(stop_msg);
                  this->goto_park();
                });
            return;
          }
          return;
        }
      }
      return;
    }
  }
  void GlobalPlanner::remove_no_entry_stations()
  {
    for (auto id : no_entry_ids)
    {
      auto it = std::find(remaining_station_ids.begin(), remaining_station_ids.end(), id);
      if (it != remaining_station_ids.end())
      {
        remaining_station_ids.erase(it);
        RCLCPP_WARN(this->get_logger(), "Station %d removed from remaining_station_ids due to no_entry.", id);
      }
    }
  }

  // Not using for now
  void GlobalPlanner::select_and_publish_nearest_station()
  {
    for (int32_t id : this->no_entry_ids)
    {
      auto lanelet_it = this->lanelet_map->laneletLayer.find(id);
      if (lanelet_it != this->lanelet_map->laneletLayer.end())
      {
        lanelet_it->attributes()["subtype"] = "no_entry";
        // RCLCPP_INFO(this->get_logger(), "Lanelet %d marked as no_entry", id);
      }
      else
      {
        // RCLCPP_WARN(this->get_logger(), "Lanelet %d not found in the map", id);
      }
    }

    if (remaining_station_ids.empty())
    {
      RCLCPP_INFO(this->get_logger(), "All stations visited.");
      RCLCPP_INFO(this->get_logger(), "Going to parking lot entry: %ld", this->parking_lot_entry_id);
      std_msgs::msg::Int32 goal_id_msg;
      goal_id_msg.data = this->parking_lot_entry_id;
      // goal_pub->publish(goal_id_msg);
      this->current_goal_station_id = this->parking_lot_entry_id;
      return;
    }

    double min_route_length = std::numeric_limits<double>::max();
    lanelet::Id nearest_id = 0;

    // Şu anki pozisyondan en yakın lanelet’i bul
    lanelet::BasicPoint2d curr_pt(this->current_position.x, this->current_position.y);
    auto nearest_lanelet = lanelet_map->laneletLayer.nearest(curr_pt, 1)[0];

    // Check if the car is currently on a station
    bool is_on_station = false;
    for (auto id : this->station_ids)
    {
      if (id == nearest_lanelet.id())
      {
        is_on_station = true;
        break;
      }
    }

    if (is_on_station)
    {
      auto it = std::find(this->remaining_station_ids.begin(), this->remaining_station_ids.end(), nearest_lanelet.id());
      if (it != this->remaining_station_ids.end())
      {
        this->remaining_station_ids.erase(it);
        RCLCPP_INFO(this->get_logger(), "Current station ID %ld removed from remaining_station_ids.", nearest_lanelet.id());
      }
    }

    for (auto id : remaining_station_ids)
    {
      auto goal_it = lanelet_map->laneletLayer.find(id);
      if (goal_it == lanelet_map->laneletLayer.end())
        continue;

      auto goal_lanelet = *goal_it;

      auto route_opt = routing_graph->getRoute(nearest_lanelet, goal_lanelet, false); // false = lane change yok
      if (!route_opt)
      {
        RCLCPP_WARN(this->get_logger(), "Route not found from %ld to %ld", nearest_lanelet.id(), id);
        continue;
      }

      double route_length = route_opt->length2d(); // yol uzunluğu (metrik)
      RCLCPP_INFO(this->get_logger(), "Route length from %ld to %ld: %f", nearest_lanelet.id(), id, route_length);

      if (route_length < min_route_length)
      {
        min_route_length = route_length;
        nearest_id = id;
      }
    }

    if (nearest_id == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "No reachable station found!");
      return;
    }

    current_goal_station_id = nearest_id;

    std_msgs::msg::Int32 goal_id_msg;
    goal_id_msg.data = nearest_id;
    goal_pub->publish(goal_id_msg);

    RCLCPP_INFO(this->get_logger(), "Selected nearest station id by route: %ld (length: %f)", nearest_id, min_route_length);
  }

  void GlobalPlanner::goto_park()
  {
    RCLCPP_INFO(this->get_logger(), "Going to parking lot entry lanelet %ld", this->parking_lot_entry_id);
    if (!this->park_go)
    {
      RCLCPP_INFO(this->get_logger(), "Going to parking lot entry: %ld", this->parking_lot_entry_id);
      this->park_go = true;
      this->do_park = true;
    }
  }

  void GlobalPlanner::on_wait_timer()
  {
    wait_timer_->cancel(); // Bir kere çalışsın
    // O durağı remaining_station_ids listesinden çıkar
    // remaining_station_ids.erase(
    //     std::remove(remaining_station_ids.begin(), remaining_station_ids.end(), current_goal_station_id),
    //     remaining_station_ids.end());
    at_goal_station = false;
    current_goal_station_id = 0;
    // Yeniden hedef seç ve git
    select_and_publish_nearest_station();
  }

  // Not using for now
  void GlobalPlanner::no_entry_arr_sub(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg)
  {
    if (this->no_entry_ids != msg->data)
    {
      RCLCPP_INFO(this->get_logger(), "no_entry_ids güncellendi: ");
      for (auto id : msg->data)
        RCLCPP_INFO(this->get_logger(), "- %d", id);

      this->no_entry_ids = msg->data;
      remove_no_entry_stations();
      goal_pose_update = true;
    }
  }

  void GlobalPlanner::cluster_callback(const cluster_msgs::msg::ClusterPointsArray &msg)
  {
    const lanelet::BasicPoint2d current_point(this->current_position.x, this->current_position.y);

    int currentId = this->lanelet_map->laneletLayer.nearest(current_point, 1)[0].id();

    if (currentId != this->parking_lot_entry_id)
    {
      return;
    }

    std::map<int, bool> isAvailable;
    for (const auto &id : this->park_ids)
    {
      isAvailable[id] = true;
    }

    std::vector<lanelet::BasicPoint2d> occupied_points;

    for (const auto &cluster : msg.clusters)
    {
      int num = 0;
      float total_x = 0.0;
      float total_y = 0.0;

      for (const auto &point : cluster.bottom_points_array)
      {
        geometry_msgs::msg::PointStamped cluster_point;
        cluster_point.header.frame_id = msg.header.frame_id; // actual lidar frame
        cluster_point.point.x = point.x;
        cluster_point.point.y = point.y;
        cluster_point.point.z = point.z;

        // Transform the point to the occupancy grid frame
        geometry_msgs::msg::PointStamped transformed_point;
        try
        {
          transformed_point = tf_buffer_.transform(cluster_point, "map");
          total_x += transformed_point.point.x;
          total_y += transformed_point.point.y;
        }
        catch (const tf2::TransformException &ex)
        {
          // RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
          continue;
        }
        num++;
      }

      float avg_x = total_x / num;
      // RCLCPP_INFO(this->get_logger(), "Average X: %f", avg_x);
      float avg_y = total_y / num;
      // RCLCPP_INFO(this->get_logger(), "Average Y: %f", avg_y);

      lanelet::BasicPoint2d avg_point;
      avg_point.x() = avg_x;
      avg_point.y() = avg_y;

      occupied_points.push_back(avg_point);
      // RCLCPP_INFO(this->get_logger(), "Cluster average point: x: %f, y: %f", avg_x, avg_y);
    }
    // OCCUPİED NOKTALARIN KONTROLÜ
    //  RCLCPP_INFO(this->get_logger(), "size of occupied points: %zu", occupied_points.size());
    for (const auto &id : this->park_ids)
    { // burası değişecek
      // RCLCPP_INFO(this->get_logger(), "Checking parking lot ID: %d", id);
      if (!isAvailable[id])
      {
        continue; // Skip if the parking lot is already occupied
      }
      // Check if the polygon intersects with the lanelet
      auto it = this->lanelet_map->laneletLayer.find(id);
      auto lanelet = *it;
      // şuraya girmiyor gibi
      for (const auto &avg_point : occupied_points)
      {
        if (lanelet::geometry::inside(lanelet, avg_point) && isAvailable[id])
        {
          // RCLCPP_INFO(this->get_logger(), "Park ID %d is occupied by a cluster point at x: %f, y: %f", id, avg_point.x(), avg_point.y());
          // If the point is inside the lanelet polygon, mark it as occupied
          isAvailable[id] = false;
        }
        else if (!isAvailable[id])
        {
          isAvailable[id] = false;
          // RCLCPP_INFO(this->get_logger(), "Park ID %d is occupied", id);
        }
        else
        {
          // RCLCPP_INFO(this->get_logger(), "Park ID %d is available", id);
        }
      }
    }

    for (const auto &[id, available] : isAvailable)
    {
      // RCLCPP_INFO(this->get_logger(), "Park ID %d is %s", id, available ? "available" : "occupied");
      if (!available)
      {
        static bool published = false;
        if (!published)
        {
          this->closed_park_ids.push_back(id);
          published = true;
        }
        return; // Publish the first available parking lot ID and exit
      }
    }
  }
} // end of namespace smart_car

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::GlobalPlanner)
