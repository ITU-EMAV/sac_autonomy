#include "object_clustering/travel_object_clustering_nodelet.hpp"
#include "unordered_set"
#include <algorithm>

namespace object_clustering {

  enum MarkerColor {
      GREEN  = 0,  // Valid cluster
      YELLOW = 1,  // Behind the front margin of the car
      RED    = 2,  // Inside the cropbox filter
      PURPLE = 3   // Above the minZ threshold
  };

// Function to project a 3D point cloud onto the XY plane by ignoring the Z axis
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectToXY(const pcl::PointCloud<PointType>::Ptr& input_cloud) {
    auto output_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto& point : input_cloud->points) {
      pcl::PointXYZ xyz_point;
      xyz_point.x = point.x;
      xyz_point.y = point.y;
      xyz_point.z = 0; // Setting Z axis to 0 to project onto XY plane
      output_cloud->points.push_back(xyz_point);
    }
    return output_cloud;
  }

  double getMinZ(const visualization_msgs::msg::Marker& marker) {
    double min_z = std::numeric_limits<double>::infinity();
    for (const auto& point : marker.points) {
      if (point.z < min_z) {
        min_z = point.z;
      }
    }
    return min_z;
  }

  double getMaxZ(const visualization_msgs::msg::Marker& marker) {
      double max_z = -std::numeric_limits<double>::infinity();  // Start with the smallest possible value
      for (const auto& point : marker.points) {
          if (point.z > max_z) {  // Check if the current point's z value is greater than the current max_z
              max_z = point.z;
          }
      }
      return max_z;
  }

  double getMaxX(const visualization_msgs::msg::Marker& marker) {
      double max_x = -std::numeric_limits<double>::infinity();  // Start with the smallest possible value
      for (const auto& point : marker.points) {
          if (point.x > max_x) {  // Check if the current point's z value is greater than the current max_z
              max_x = point.x;
          }
      }
      return max_x;
  }

  bool cropboxFilter(const std::vector<geometry_msgs::msg::Point>& points, 
                           double car_lat_margin, 
                           double car_front_margin) {
    for (const auto& point : points) {
      if (point.x < car_front_margin && std::abs(point.y) < car_lat_margin) { 
        return false;
      }
    }
    return true;
  }

  visualization_msgs::msg::Marker copy_marker(const visualization_msgs::msg::Marker& marker_, MarkerColor color_) {
    visualization_msgs::msg::Marker m_marker;

    m_marker.header.frame_id = marker_.header.frame_id;
    m_marker.header.stamp = marker_.header.stamp;
    m_marker.ns = marker_.ns;
    m_marker.id = marker_.id;
    m_marker.type = marker_.type;
    m_marker.action = marker_.action;
    m_marker.pose.orientation.w = marker_.pose.orientation.w;
    m_marker.scale.x = marker_.scale.x; // Adjust the line width as needed
    m_marker.points = marker_.points;

    if (color_ == MarkerColor::GREEN) {
      m_marker.color.a = 1.0; // Don't forget to set the alpha!
      m_marker.color.r = 0.0;
      m_marker.color.g = 1.0;
      m_marker.color.b = 0.0;
    } 
    else if (color_ == MarkerColor::YELLOW) {
      m_marker.color.a = 1.0; // Don't forget to set the alpha!
      m_marker.color.r = 1.0;
      m_marker.color.g = 1.0;
      m_marker.color.b = 0.0;
    }
    else if (color_ == MarkerColor::RED) {
      m_marker.color.a = 1.0; // Don't forget to set the alpha!
      m_marker.color.r = 1.0;
      m_marker.color.g = 0.0;
      m_marker.color.b = 0.0;
    }
    else if (color_ == MarkerColor::PURPLE) {
      m_marker.color.a = 1.0; // Don't forget to set the alpha!
      m_marker.color.r = 0.5;
      m_marker.color.g = 0.0;
      m_marker.color.b = 0.5;
    }

    return m_marker;
  }

  struct PointHasher {  // Written as functors to get the type of the object correctly in compile time
    std::size_t operator()(const geometry_msgs::msg::Point& p) const {
      std::size_t h1 = std::hash<double>()(p.x);
      std::size_t h2 = std::hash<double>()(p.y);
      std::size_t h3 = std::hash<double>()(p.z);
      return h1 ^ h2 ^ h3;
    }
  };

// Custom equality comparator for geometry_msgs::msg::Point
  struct PointComparator { // It is functor, not a function. Functors are generally inlined and stateful
    bool operator()(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const {
      return (fabs(p1.x - p2.x) < 1e-6) && (fabs(p1.y - p2.y) < 1e-6) && (fabs(p1.z - p2.z) < 1e-6);
    }
  };

  std::vector<geometry_msgs::msg::Point> getBottomSurfacePoints(const visualization_msgs::msg::Marker& marker) {
    double min_z = getMinZ(marker);

    // Use unordered_set to store unique points with custom hash and comparator
    std::unordered_set<geometry_msgs::msg::Point, PointHasher, PointComparator> unique_points;

    // Add points with the minimum z value to the set
    for (const auto& point : marker.points) {
      if (fabs(point.z - min_z) < 1e-6) { // Use a small epsilon for floating-point comparison
        unique_points.insert(point);
      }
    }

    // Convert the set of unique points to a vector and return
    return std::vector<geometry_msgs::msg::Point>(unique_points.begin(), unique_points.end());
  }

  TravelObjectClusteringComponent::TravelObjectClusteringComponent(const rclcpp::NodeOptions& options)
          : rclcpp::Node("TravelObjectClustering", options) {

    // Create a custom QoS profile
    rclcpp::QoS custom_qos = rclcpp::QoS(rclcpp::KeepLast(1))  // latest collision state only
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)  // Ensure all messages are delivered
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // Message is lost if the publisher goes down

    // Retrieve debug_mode parameter
    this->declare_parameter("debug_mode", false);
    this->declare_parameter("can_see_back", false);
    bool debug_mode = this->get_parameter("debug_mode").as_bool();
    can_see_back = this->get_parameter("can_see_back").as_bool();
    
    // Declare and get parameters
    MIN_RANGE = this->declare_parameter<float>("min_range", 1.0);
    MAX_RANGE = this->declare_parameter<float>("max_range", 64.0);
    FRONT_RANGE = this->declare_parameter<float>("front_range", 30.0);
    REAR_RANGE = this->declare_parameter<float>("rear_range", 10.0);
    LATERAL_RANGE = this->declare_parameter<float>("lateral_range", 10.0);
    FRAME_ID = this->declare_parameter<std::string>("frame_id", "base_link");
    INPUT_TOPIC = this->declare_parameter<std::string>("input_topic", "nonground");
    VERT_SCAN = this->declare_parameter<int>("vert_scan", 10);
    HORZ_SCAN = this->declare_parameter<int>("horz_scan", 20);
    CAR_TOP_MARGIN = this->declare_parameter<float>("car_top_margin", 0.2);
    CAR_LAT_MARGIN = this->declare_parameter<float>("car_lat_margin", 0.5);
    CAR_FRONT_MARGIN = this->declare_parameter<float>("car_front_margin", 0.3);
    VERT_MERGE_THRES = this->declare_parameter<float>("th_vert_merg", 0.2);
    HORZ_MERGE_THRES = this->declare_parameter<float>("th_horz_merg", 0.1);
    VERT_SCAN_SIZE = this->declare_parameter<int>("vert_scan_size", 10);
    HORZ_SCAN_SIZE = this->declare_parameter<int>("horz_scan_size", 20);
    HORZ_SKIP_SIZE = this->declare_parameter<int>("horz_skip_size", 20);
    HORZ_EXTENSION_SIZE = this->declare_parameter<int>("horz_extension_size", 10);
    DOWNSAMPLE = this->declare_parameter<int>("downsample", 1);
    MIN_VERT_ANGLE = this->declare_parameter<float>("min_vert_angle", -15.0);
    MAX_VERT_ANGLE = this->declare_parameter<float>("max_vert_angle", 15.0);
    MIN_CLUSTER_SIZE = this->declare_parameter<int>("min_cluster_size", 30);
    MAX_CLUSTER_SIZE = this->declare_parameter<int>("max_cluster_size", 1000);
    MIN_CLUSTER_HEIGHT = this->declare_parameter<float>("min_cluster_height", 0.2);
    MAX_BOTTOM_Z = this->declare_parameter<float>("max_bottom_z", 0.3);
    // Default is effectively no crop; the vehicle profile is set in the config.
    MAX_POINT_Z = this->declare_parameter<float>("max_point_z", 100.0);


    resolution = static_cast<float>(MAX_VERT_ANGLE - MIN_VERT_ANGLE) / static_cast<float>(VERT_SCAN - 1);
    for(int i = 0; i < VERT_SCAN; i++)
      vert_angles_.push_back(MIN_VERT_ANGLE + i*resolution);

    range_mat_.resize(VERT_SCAN, std::vector<Point>(HORZ_SCAN));

    valid_cnt_.resize(VERT_SCAN, 0);
    nodes_.resize(VERT_SCAN, std::vector<AOSNode>());

    marker_array_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
    // Setup publisher and subscriber
    pc_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            INPUT_TOPIC, rclcpp::SensorDataQoS(), std::bind(&TravelObjectClusteringComponent::pointCloudCallback, this, std::placeholders::_1));

    pc_labeled_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("labeled_cloud", rclcpp::SensorDataQoS());
    cluster_polygon_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_marker", rclcpp::SensorDataQoS());
    // TODO: Test the QoS effect
    cluster_bottom_arr_publisher_ = this->create_publisher<cluster_msgs::msg::ClusterPointsArray>("all_bottom_clusters", custom_qos);

    // Show parameters always
    show_parameters();

    // Set log level based on debug_mode
    rcutils_ret_t ret;
    if (!debug_mode) {
      ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level");
      }
    }
  }

  void TravelObjectClusteringComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud){
    // Clear the point clouds

    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_pc(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr labeled_pc(new pcl::PointCloud<PointType>);

    // Update stamp_ to current timestamp
    stamp_ = point_cloud->header.stamp;

    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*point_cloud, *cloud_in);

    RCLCPP_INFO(this->get_logger(), "+--------------------------------------+");
    RCLCPP_INFO(this->get_logger(), "Received point cloud");

    // Filter NaN points and points out of range
    filtered_pc->header = cloud_in->header;
    filtered_pc->points.reserve(cloud_in->points.size());
    for (auto &point : cloud_in->points) {
      bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
      if (is_nan) {
        continue;
      }
      const float range_squared = point.x * point.x + point.y * point.y + point.z * point.z;
      if (range_squared <= MIN_RANGE * MIN_RANGE ||
          point.x > FRONT_RANGE || point.x < -REAR_RANGE ||
          std::abs(point.y) > LATERAL_RANGE) {
        continue;
      }
      // The vehicle passes under anything above this height, so overhanging
      // branches must be dropped before clustering: otherwise they merge with
      // the trunk and the whole tree blocks the lane. The trunk below the crop
      // still forms its own cluster and stays an obstacle.
      if (point.z > MAX_POINT_Z) {
        continue;
      }
      filtered_pc->push_back(point);
    }

    // Create a MarkerArray to publish all markers together
    visualization_msgs::msg::MarkerArray delete_all_markers;

    if (!marker_array_->markers.empty()){
      // Create a single delete all marker
      visualization_msgs::msg::Marker delete_all_marker;
      delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;

      // Use the same frame_id and namespace
      delete_all_marker.header = marker_array_->markers.front().header;
      delete_all_marker.ns = marker_array_->markers.front().ns;

      delete_all_markers.markers.push_back(delete_all_marker);
    } else {
      RCLCPP_WARN(this->get_logger(), "marker_array_ is empty");
    }
    // Publish the delete MarkerArray
    cluster_polygon_publisher_->publish(delete_all_markers);
    marker_array_->markers.clear();

    // Apply above-ground object segmentation
    segmentObjects(filtered_pc, labeled_pc);
    RCLCPP_INFO(this->get_logger(), "Labeled cloud size: %ld", labeled_pc->size());
          
    // Publish the labeled cloud
    sensor_msgs::msg::PointCloud2 labeled_cloud_msg;
    pcl::toROSMsg(*labeled_pc, labeled_cloud_msg);
    labeled_cloud_msg.header.frame_id = FRAME_ID;
    labeled_cloud_msg.header.stamp = point_cloud->header.stamp;
    pc_labeled_cloud_publisher_->publish(labeled_cloud_msg);

    std::vector<geometry_msgs::msg::Point> all_bottom_surface_points;

    // Create the main message to publish
    cluster_msgs::msg::ClusterPointsArray all_clusters_msg;

     // Temporary vector to hold the modified markers
    std::vector<visualization_msgs::msg::Marker> new_markers;

    // Iterate through all markers in the MarkerArray
    for (const auto& marker : marker_array_->markers) {
      bool cluster_invalid_flag = false;

      // Get the points on the bottom surface
      std::vector<geometry_msgs::msg::Point> bottom_surface_points = getBottomSurfacePoints(marker);

      // The output frame is the Velodyne frame. Ignore clusters whose lowest
      // point is above the configured height relative to the sensor origin.
      if (getMinZ(marker) > MAX_BOTTOM_Z) {
        continue;
      }

      // Determine the color based on conditions
      MarkerColor color = MarkerColor::GREEN;  // Default color

      if (getMinZ(marker) > CAR_TOP_MARGIN || getMaxZ(marker) - getMinZ(marker) < 0.2) {
        RCLCPP_DEBUG(this->get_logger(), "Min Z is too much or vertical width is too low of the cluster with id %d is %.3f", marker.id, getMinZ(marker));
        color = MarkerColor::PURPLE;
      } else if (!cropboxFilter(bottom_surface_points, CAR_LAT_MARGIN, CAR_FRONT_MARGIN)) {
        RCLCPP_DEBUG(this->get_logger(), "Cluster %d intersects the cropbox of the car", marker.id);
        color = MarkerColor::RED;
      } else {
        if ((getMaxX(marker) < CAR_FRONT_MARGIN) && !can_see_back) {
          RCLCPP_DEBUG(this->get_logger(), "Cluster %d is behind the ego vehicle", marker.id);
          color = MarkerColor::YELLOW;
        }
      }

      // Copy marker and apply the color change if needed
      visualization_msgs::msg::Marker new_marker = copy_marker(marker, color);

      // Add the new marker to the vector
      new_markers.push_back(new_marker);

      if (!cluster_invalid_flag) {
        // Create the ClusterPointsArray message
        cluster_msgs::msg::ClusterPoints bottom_points_array_msg;
        bottom_points_array_msg.cluster_id = marker.id;
        bottom_points_array_msg.bottom_points_array = bottom_surface_points;

        // Add to the all clusters message
        all_clusters_msg.clusters.push_back(bottom_points_array_msg);

        // Append to the all bottom surface points for some other usage (if necessary)
        all_bottom_surface_points.insert(all_bottom_surface_points.end(), bottom_surface_points.begin(), bottom_surface_points.end());
      }
    }

    // Replace the original markers with the modified markers
    marker_array_->markers = new_markers;

    // Set the header for all_clusters_msg
    all_clusters_msg.header.stamp = point_cloud->header.stamp;
    all_clusters_msg.header.frame_id = FRAME_ID;

    // Publish the all clusters message
    cluster_bottom_arr_publisher_->publish(all_clusters_msg);

    // Publish the MarkerArray
    RCLCPP_INFO(this->get_logger(), "Publishing %ld markers", marker_array_->markers.size());
    RCLCPP_INFO(this->get_logger(), "+--------------------------------------+");

    cluster_polygon_publisher_->publish(*marker_array_);

    marker_id = 0;
  }

  void TravelObjectClusteringComponent::segmentObjects(pcl::PointCloud<PointType>::Ptr cloud_in,
                        pcl::PointCloud<PointType>::Ptr cloud_out){
    // 0. reset
    max_label_ = 1;
    clusters_.clear();
    clusters_.push_back(empty_);
    clusters_.push_back(empty_);
    std::fill(valid_cnt_.begin(), valid_cnt_.end(), 0);

    std::for_each(range_mat_.begin(), range_mat_.end(), [](std::vector<Point>& inner_vec) {
      std::fill(inner_vec.begin(), inner_vec.end(), Point());
    });

    // 1. do spherical projection
    auto start = std::chrono::steady_clock::now();
    sphericalProjection(cloud_in);
    auto end = std::chrono::steady_clock::now();
    // cloud_out->points.resize(cloud_in->points.size());
    RCLCPP_INFO(this->get_logger(), "Creating range map: %.2f ms", std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end-start).count());

    // 2. begin clustering
    start = std::chrono::steady_clock::now();
    for (int channel = 0; channel < VERT_SCAN; channel++) {
      horizontalUpdate(channel);
      verticalUpdate(channel);
    }
    end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Processing clusters: %.2f ms", std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end-start).count());

    // 3. post-processing (clusters -> labels)
    start = std::chrono::steady_clock::now();
    labelPointcloud(cloud_in, cloud_out);
    end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Labeling clusters: %.2f ms", std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end-start).count());
  }

  void TravelObjectClusteringComponent::sphericalProjection(pcl::PointCloud<PointType>::Ptr cloud_in) {
    pcl::PointCloud<PointType>::Ptr valid_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    valid_cloud->points.reserve(cloud_in->points.size());
    int ring_idx = -1, row_idx = -1, col_idx = -1;
    float range;
    Point point;
    int range_chcker = 0;
    int valid_checker = 0;
    int count_range, count_downsample, count_out_of_row, count_out_of_col, count_out_of_mat;
    count_range = count_downsample = count_out_of_row = count_out_of_col = count_out_of_mat = 0;
    int count_pass = 0;
    for (size_t i = 0; i < cloud_in->points.size(); i++) {

      range = getRange(cloud_in->points[i]);
      if (range < MIN_RANGE || range > MAX_RANGE){
        count_range++;
        continue;
      }
      row_idx = getRowIdx(cloud_in->points[i]);

      if (row_idx % DOWNSAMPLE != 0) {
        count_downsample++;
        continue;
      }
      if (row_idx < 0 || row_idx >= VERT_SCAN) {
        count_out_of_row++;
        continue;
      }
      col_idx = getColIdx(cloud_in->points[i]);
      if (col_idx < 0 || col_idx >= HORZ_SCAN) {
        count_out_of_col++;
        continue;
      }
      if (range_mat_[row_idx][col_idx].valid){
        count_out_of_mat ++;
        continue;
      } else {
        count_pass++;
        point.x = cloud_in->points[i].x;
        point.y = cloud_in->points[i].y;
        point.z = cloud_in->points[i].z;
        point.valid = true;
        point.idx = valid_cloud->points.size();
        valid_cloud->points.push_back(cloud_in->points[i]);
        range_mat_[row_idx][col_idx] = point;
        assert(range_mat_[row_idx][col_idx].valid == true);
        valid_cnt_[row_idx]++;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Input cloud size: %d", (int)cloud_in->points.size());
    RCLCPP_INFO(this->get_logger(), "Projected cloud size: %d", (int)valid_cloud->points.size());
    *cloud_in = *valid_cloud;
  }

  void TravelObjectClusteringComponent::labelPointcloud(pcl::PointCloud<PointType>::Ptr cloud_in,
                       pcl::PointCloud<PointType>::Ptr cloud_out){
    uint cnt = 0;  // number of valid clusters, just excluding empty cluster
    uint pt_cnt = 0;  // number of points in valid clusters
    uint valid_cluster_size = 0;
    cloud_out->points.reserve(cloud_in->points.size());

    // generate random number
    std::vector<size_t> valid_indices;
    for (size_t i = 0; i < clusters_.size(); i++) {
      if (std::distance(clusters_[i].begin(), clusters_[i].end()) > 0) {  // check if cluster is not empty
        valid_indices.push_back(i);
        cnt++;
      }
    }
    std::list<uint16_t> l(cnt);
    std::iota(l.begin(), l.end(), 1);
    std::vector<std::list<uint16_t>::iterator> v(l.size());
    std::iota(v.begin(), v.end(), l.begin());
    std::shuffle(v.begin(), v.end(), std::mt19937{std::random_device{}()});

    for (size_t i = 0; i < valid_indices.size(); i++) {
      uint16_t label = *v[i];  // random label, basically a number from 1 to cnt
      size_t idx = valid_indices[i];
      int point_size = std::distance(clusters_[idx].begin(), clusters_[idx].end());  // number of points in the cluster
      if (MIN_CLUSTER_SIZE && MAX_CLUSTER_SIZE) {
        if (point_size < MIN_CLUSTER_SIZE || point_size > MAX_CLUSTER_SIZE)
          continue;
      }
      valid_cluster_size++;

      pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>);
      for (auto &p : clusters_[idx]) {
        if (p->valid) {
          assert(cloud_in->points[p->idx].x == p->x);
          assert(cloud_in->points[p->idx].y == p->y);
          assert(cloud_in->points[p->idx].z == p->z);
          PointType point = cloud_in->points[p->idx];
          point.intensity = label;
          cloud_out->points.push_back(point);
          cluster_cloud->points.push_back(point);
          pt_cnt++;
        } else {
          RCLCPP_ERROR(this->get_logger(), "point invalid");
        }
      }
      // Project points to XY plane
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_xy = projectToXY(cluster_cloud);

      if (cluster_cloud_xy->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "cluster_cloud_xy not proper");
      }

      // TODO: Known Issues:
      // 1. Object clustering dusey olarak duz cizgi cluster atiyor ve XY duzleminde nokta olarak bir Convex Hull
      //    olusturuluyor. Bu da asagidaki warning'e neden oluyor:
      //      [component_container_mt-1] QH7089 qhull precision warning: The initial hull is narrow.  Is the input lower
      //      [component_container_mt-1] dimensional (e.g., a square in 3-d instead of a cube)?  Cosine of the minimum
      //      [component_container_mt-1] angle is 0.9999999999999998.  If so, Qhull may produce a wide facet.
      //      [component_container_mt-1] Options 'Qs' (search all points), 'Qbb' (scale last coordinate), or
      //      [component_container_mt-1] 'QbB' (scale to unit box) may remove this warning.
      //      [component_container_mt-1] See 'Limitations' in qh-impre.htm.  Use 'Pp' to skip this warning.

      // 2. Horizontal update ve merge islemi icin parametre arasitrmasi yapilmasi lazim

      // Compute 2D convex hull using PointXYZ
      if (cluster_cloud_xy->points.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Cluster with label %d has less than 3 points, skipping convex hull computation", label);
        continue; // Skip clusters with less than 3 points
      }
      pcl::ConvexHull<pcl::PointXYZ> convex_hull;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_xy(new pcl::PointCloud<pcl::PointXYZ>);
      convex_hull.setInputCloud(cluster_cloud_xy);
      convex_hull.setDimension(2); // Force the convex hull to be computed in 2D
      convex_hull.reconstruct(*cloud_hull_xy);

      // Find min and max Z values
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
      for (const auto& point : cluster_cloud->points) {
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
      }
      if ((max_z - min_z) < MIN_CLUSTER_HEIGHT) {
        // RCLCPP_WARN(this->get_logger(), "Cluster with height %.2f m is below threshold %.2f m. Skipping.", (max_z - min_z), MIN_CLUSTER_HEIGHT);
        continue; // don't create a bounding box for clusters that are too flat
      }


      // Create 3D bounding box by extruding 2D convex hull
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = FRAME_ID;
      marker.header.stamp = stamp_; // current timestamp taken from initial input cloud
      marker.ns = "cluster_bounding_boxes";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2; // Adjust the line width as needed
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      // Add bottom edges
      for (size_t j = 0; j < cloud_hull_xy->points.size(); ++j) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = cloud_hull_xy->points[j].x;
        p1.y = cloud_hull_xy->points[j].y;
        p1.z = min_z;
        p2.x = cloud_hull_xy->points[(j + 1) % cloud_hull_xy->points.size()].x;
        p2.y = cloud_hull_xy->points[(j + 1) % cloud_hull_xy->points.size()].y;
        p2.z = min_z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }

      // Add top edges
      for (size_t j = 0; j < cloud_hull_xy->points.size(); ++j) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = cloud_hull_xy->points[j].x;
        p1.y = cloud_hull_xy->points[j].y;
        p1.z = max_z;
        p2.x = cloud_hull_xy->points[(j + 1) % cloud_hull_xy->points.size()].x;
        p2.y = cloud_hull_xy->points[(j + 1) % cloud_hull_xy->points.size()].y;
        p2.z = max_z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }

      // Add vertical edges
      for (auto & point : cloud_hull_xy->points) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = point.x;
        p1.y = point.y;
        p1.z = min_z;
        p2.x = point.x;
        p2.y = point.y;
        p2.z = max_z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
      this->marker_array_->markers.push_back(marker);
    }

    RCLCPP_INFO(this->get_logger(), "Number of clusters: %d, points: %d", cnt, pt_cnt);
    RCLCPP_INFO(this->get_logger(), "Valid cluster size: %d", valid_cluster_size);
  }

  void TravelObjectClusteringComponent::horizontalUpdate(int channel) {
    nodes_[channel].clear();
    // no point in the channel
    if (valid_cnt_[channel] <= 0){
      return;
    }

    int first_valid_idx = -1;
    int last_valid_idx = -1;
    int start_pos = -1;
    int pre_pos = -1;
    int end_pos = -1;

    AOSNode node;
    bool first_node = false;
    for(int j = 0; j < HORZ_SCAN; j++) {
      if (!range_mat_[channel][j].valid)
        continue;
      if (!first_node) {
        first_node = true;
        // update index
        first_valid_idx = j;
        start_pos = j;
        pre_pos = j;
        // update label
        max_label_++;
        clusters_.push_back(empty_);
        range_mat_[channel][j].label = max_label_;
        // push a new node
        node.start = start_pos;
        node.end = j;
        node.label = range_mat_[channel][j].label;
        nodes_[channel].push_back(node);
        clusters_[node.label].insert_after(clusters_[node.label].cbefore_begin(), &range_mat_[channel][j]);
      } else {
        auto &cur_pt = range_mat_[channel][j];
        auto &pre_pt = range_mat_[channel][pre_pos];
        if (pointDistance(cur_pt, pre_pt) < HORZ_MERGE_THRES) {
          // update existing node
          pre_pos = j;
          cur_pt.label = pre_pt.label;
          nodes_[channel].back().end = j;
        } else {
          // update index
          start_pos = j;
          pre_pos = j;
          // update label
          max_label_++;
          clusters_.push_back(empty_);
          cur_pt.label = max_label_;
          // push new node
          node.start = start_pos;
          node.end = j;
          node.label = range_mat_[channel][j].label;
          nodes_[channel].push_back(node);
          assert(range_mat_[channel][j].label == cur_pt.label);
        }
        clusters_[cur_pt.label].insert_after(clusters_[cur_pt.label].cbefore_begin(), &range_mat_[channel][j]);
      }
    }
    last_valid_idx = pre_pos;

    // merge last and first points
    if (nodes_[channel].size() > 2) {
      auto &p_0 = range_mat_[channel][first_valid_idx];
      auto &p_l = range_mat_[channel][last_valid_idx];
      if (pointDistance(p_0, p_l) < HORZ_MERGE_THRES) {
        if (p_0.label == 0)
          RCLCPP_WARN(this->get_logger(), "Ring merge to label 0");
        if (p_0.label != p_l.label) {
          nodes_[channel].back().label = p_0.label;
          mergeClusters(p_l.label, p_0.label);
        }
      }
    }

    // merge skipped nodes due to occlusion
    if (nodes_[channel].size() > 2) {
      uint16_t cur_label = 0, target_label = 0;
      for (size_t i = 0; i < nodes_[channel].size()-1; i++){
        for (size_t j = i+1; j < nodes_[channel].size(); j++) {
          auto &node_i = nodes_[channel][i];
          auto &node_j = nodes_[channel][j];

          if (node_i.label == node_j.label)
            continue;

          int end_idx = node_i.end;
          int start_idx = node_j.start;
          float dist = pointDistance(range_mat_[channel][end_idx], range_mat_[channel][start_idx]);
          if (dist < HORZ_MERGE_THRES) {
            if (node_i.label > node_j.label) {
              target_label = node_j.label;
              cur_label = node_i.label;
              node_i.label = target_label;
            } else {
              target_label = node_i.label;
              cur_label = node_j.label;
              node_j.label = target_label;
            }
            mergeClusters(cur_label, target_label);
          }
          if (j-i >= HORZ_SKIP_SIZE)
            break;
        }
      }
    }
  }

  void TravelObjectClusteringComponent::verticalUpdate(int channel) {
    // Iterate each point of this channel to update the labels.
    int point_size = valid_cnt_[channel];
    // Current scan line is emtpy, do nothing.
    if(point_size == 0) return;
    // Skip first scan line
    if(channel == 0) return;

    int prev_channel = channel - 1;

    for(int n = 0; n < nodes_[channel].size(); n++) {

      auto &cur_node = nodes_[channel][n];

      for (int l = prev_channel; l >= channel - VERT_SCAN_SIZE; l -= 1) {

        if (l < 0) // out of range
          break;

        if (valid_cnt_[l] == 0) {
          continue;
        }

        // binary search lower bound
        // lower_bnd inclusive
        int N = nodes_[l].size();
        int first = 0;
        int last = N - 1;
        int lower_bnd = 0;
        int mid = 0;

        while (first <= last) {
          mid = (first + last) / 2;
          auto &prev_node = nodes_[l][mid];
          if (overlap(cur_node, prev_node) || cur_node.end < prev_node.start){
            lower_bnd = mid;
            last = mid - 1;
          } else {
            first = mid + 1;
          }
        }

        // binary search upper bound
        // exclusive but gives -1 if end of list
        first = 0;
        last = N - 1;
        mid = 0;
        int upper_bnd = 0;

        while (first <= last) {
          mid = (first + last) / 2;
          auto &prev_node = nodes_[l][mid];
          if (overlap(cur_node, prev_node) || prev_node.end < cur_node.start) {
            upper_bnd = mid;
            first = mid + 1;
          } else {
            last = mid - 1;
          }
        }
        upper_bnd = upper_bnd + 1;

        // loop through overlapped nodes
        for (size_t idx = lower_bnd; idx < upper_bnd; idx++) {

          auto &ovl_node = nodes_[l][idx];

          if (ovl_node.label == cur_node.label) {
            continue;
          }

          int iter_start_idx = -1;
          int iter_end_idx = -1;

          if (ovl_node.start <= cur_node.start && cur_node.end <= ovl_node.end) {
            // cur_node inside prev_node
            iter_start_idx = cur_node.start;
            iter_end_idx = cur_node.end;
          } else if (cur_node.start <= ovl_node.start && ovl_node.end <= cur_node.end) {
            // prev_node inside cur_node
            iter_start_idx = ovl_node.start;
            iter_end_idx = ovl_node.end;
          } else if (cur_node.start < ovl_node.start && cur_node.end >= ovl_node.start && cur_node.end <= ovl_node.end) {
            // tail of cur_node with head of prev_node
            iter_start_idx = ovl_node.start;
            iter_end_idx = cur_node.end;
          } else if (ovl_node.start <= cur_node.start && cur_node.start <= ovl_node.end && cur_node.end > ovl_node.end) {
            // head of cur_node with tail of prev_node
            iter_start_idx = cur_node.start;
            iter_end_idx = ovl_node.end;
          } else {
            continue;
          }

          // iterate through overlapping indices
          uint16_t cur_label = 0, target_label = 0;
          bool merged = false;
          int cur_start_left = iter_start_idx;
          int cur_start_right = iter_start_idx;
          int cur_end_left = iter_end_idx;
          int cur_end_right = iter_end_idx;

          while (true) {
            if (cur_start_right > cur_end_left && cur_start_left < iter_start_idx - HORZ_SCAN_SIZE && cur_end_right > iter_end_idx + HORZ_SCAN_SIZE) // end of search
              break;
            if (mergeNodes(cur_node, ovl_node, channel, l, cur_start_left))
              break;
            if (cur_start_left != cur_end_left) { // more than one overlapping cur_node point
              if (mergeNodes(cur_node, ovl_node, channel, l, cur_end_left))
                break;
            }
            if (cur_start_left != cur_start_right) { // not the first iteration
              if (mergeNodes(cur_node, ovl_node, channel, l, cur_start_right))
                break;
            }
            if (cur_end_left != cur_end_right) { // not the first iteration
              if (mergeNodes(cur_node, ovl_node, channel, l, cur_end_right))
                break;
            }
            cur_start_left--;
            cur_start_right++;
            cur_end_left--;
            cur_end_right++;
          }
        }
      }
    }
  }

  bool TravelObjectClusteringComponent::overlap(AOSNode &first_node, AOSNode &second_node) const {
    int new_start = first_node.start - HORZ_EXTENSION_SIZE;
    int new_end = first_node.end + HORZ_EXTENSION_SIZE;
    if (new_start <= second_node.start && second_node.start <= new_end)
      return true;
    else if (new_start <= second_node.end && second_node.end <= new_end)
      return true;
    else if (second_node.start <= new_start && new_end <= second_node.end)
      return true;
    return false;
  }

  bool TravelObjectClusteringComponent::mergeNodes(AOSNode &first_node, AOSNode &second_node, int cur_channel, int prev_channel, int query_idx) {
    if (query_idx >= first_node.start && query_idx <= first_node.end) {
      if (range_mat_[cur_channel][query_idx].valid) {
        int left_idx = query_idx;
        int right_idx = query_idx;
        while (1) {
          if (left_idx <= query_idx - HORZ_SCAN_SIZE && right_idx >= query_idx + HORZ_SCAN_SIZE)
            break;

          if (left_idx >= second_node.start && left_idx <= second_node.end && range_mat_[prev_channel][left_idx].valid) {
            if (nodeDistance(first_node, second_node, range_mat_[cur_channel][query_idx], range_mat_[prev_channel][left_idx])) {
              return true;
            }
          }

          if (right_idx <= second_node.end && right_idx >= second_node.start && range_mat_[prev_channel][right_idx].valid) {
            if (nodeDistance(first_node, second_node, range_mat_[cur_channel][query_idx], range_mat_[prev_channel][right_idx])) {
              return true;
            }
          }
          left_idx--;
          right_idx++;
        }
      }
    }
    return false;
  }

  bool TravelObjectClusteringComponent::nodeDistance(AOSNode &first_node, AOSNode &second_node, Point &first_point, Point &second_point) {
    uint16_t cur_label, target_label = 0;

    if (first_node.label == second_node.label)
      return false;

    if (pointDistance(first_point, second_point) < VERT_MERGE_THRES) {
      if (first_node.label > second_node.label) {
        cur_label = first_node.label;
        target_label = second_node.label;
        first_node.label = target_label;
      } else {
        cur_label = second_node.label;
        target_label = first_node.label;
        second_node.label = target_label;
      }
      mergeClusters(cur_label, target_label);
      return true;
    }
    return false;
  }

  float TravelObjectClusteringComponent::getRange(PointType pt) {
    return sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
  }

  int TravelObjectClusteringComponent::getRowIdx(PointType pt) {
    float angle = atan2(pt.z, sqrt(pt.x*pt.x+pt.y*pt.y))*180/M_PI;
    auto iter_geq = std::lower_bound(vert_angles_.begin(), vert_angles_.end(), angle);
    int row_idx;

    if (iter_geq == vert_angles_.begin()) {
      row_idx = 0;
    } else {
      float a = *(iter_geq - 1);
      float b = *(iter_geq);
      if (fabs(angle-a) < fabs(angle-b)){
        row_idx = iter_geq - vert_angles_.begin() - 1;
      } else {
        row_idx = iter_geq - vert_angles_.begin();
      }
    }
    return row_idx;
  }

  [[maybe_unused]] int TravelObjectClusteringComponent::getRowIdx(PointType pt, std::vector<float> vert_angles) {
    float angle = atan2(pt.z, sqrt(pt.x*pt.x+pt.y*pt.y))*180/M_PI;
    auto iter_geq = std::lower_bound(vert_angles.begin(), vert_angles.end(), angle);
    int row_idx;

    if (iter_geq == vert_angles.begin()) {
      row_idx = 0;
    } else {
      float a = *(iter_geq - 1);
      float b = *(iter_geq);
      if (fabs(angle-a) < fabs(angle-b)){
        row_idx = iter_geq - vert_angles.begin() - 1;
      } else {
        row_idx = iter_geq - vert_angles.begin();
      }
    }
    return row_idx;
  }

  int TravelObjectClusteringComponent::getColIdx(PointType pt) const{
    float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
    static float ang_res_x = 360.0/float(HORZ_SCAN);
    int col_idx = -round((horizonAngle-90.0)/ang_res_x) + HORZ_SCAN/2;
    if (col_idx >= HORZ_SCAN)
      col_idx -= HORZ_SCAN;
    return col_idx;
  }

  void TravelObjectClusteringComponent::mergeClusters(uint16_t cur_label, uint16_t target_label) {
    if(cur_label == 0 || target_label == 0){
      RCLCPP_ERROR(this->get_logger(), "Error merging runs cur_label:%u target_label:%u", cur_label, target_label);
    }
    for(auto &p : clusters_[cur_label]){
      p->label = target_label;
    }
    clusters_[target_label].insert_after(clusters_[target_label].cbefore_begin(), clusters_[cur_label].begin(),clusters_[cur_label].end() );
    clusters_[cur_label].clear();
  }

} // namespace object_clustering
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_clustering::TravelObjectClusteringComponent)
