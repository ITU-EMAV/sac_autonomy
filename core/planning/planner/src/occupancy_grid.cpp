#include "occupancy_grid.hpp"
    
// TODO: Implement the updation of the occupancy grid based on lanelet boundaries.
// TODO: Implement message filters to synchronize the localization and cluster messages.
// TODO: There are some bugs with the turns. I do not know it is related to lanelet. And there is a need for recovering from if the vehicle is outside of the lanes due to poor control or etc.
// TODO: Right and left lanelets should be free since lane change is ok.
namespace smart_car{
    OccupancyGrid::OccupancyGrid(const rclcpp::NodeOptions & options) 
        : Node("occupancy_grid_exe", options), 
          tf_buffer_(this->get_clock()), 
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "occupancy_grid is initialized!");

        this->declare_parameter<double>("grid_resolution", 0.1);
        this->declare_parameter<int>("grid_size", 100);

        this->declare_parameter<double>("origin_pose._x", 0.0);
        this->declare_parameter<double>("origin_pose._y", 0.0);
        this->declare_parameter<std::string>("osm_path", "./map.osm");

        this->declare_parameter<std::string>("pose_topic", "/localization/ndt_pose");
        this->declare_parameter<std::string>("occupancy_grid_topic", "/occupancy_grid");
        this->declare_parameter<std::string>("cluster_topic", "/all_bottom_clusters");
        this->declare_parameter<std::string>("path_topic", "/path_ids");

        this->declare_parameter<double>("vehicle_width", 1.8); // Vehicle width (in meters)
        this->declare_parameter<double>("vehicle_length", 4.2); // Vehicle length (in meters)
        this->declare_parameter<double>("safety_margin", 0.3); // Safety margin (in meters)        

        this->declare_parameter<bool>("debug_mode", false);

        this->grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        this->grid_size_ = this->get_parameter("grid_size").as_int();
        this->osm_path  = this->get_parameter("osm_path").as_string();

        this->origin_x = this->get_parameter("origin_pose._x").as_double();
        this->origin_y = this->get_parameter("origin_pose._y").as_double();

        this->vehicle_width = this->get_parameter("vehicle_width").as_double();
        this->vehicle_length = this->get_parameter("vehicle_length").as_double();
        this->safety_margin = this->get_parameter("safety_margin").as_double();

        this->debug_mode = this->get_parameter("debug_mode").as_bool();

        std::string OCCUPANCY_GRID_TOPIC = this->get_parameter("occupancy_grid_topic").as_string();
        std::string CLUSTER_TOPIC = this->get_parameter("cluster_topic").as_string();
        std::string PATH_TOPIC = this->get_parameter("path_topic").as_string();
        std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
        
        this->readLaneletMap();

        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(OCCUPANCY_GRID_TOPIC, rclcpp::QoS(1));
        
        lanelet_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/debug/lanelet_markers", 10);

        cluster_sub_ = this->create_subscription<cluster_msgs::msg::ClusterPointsArray>(
            CLUSTER_TOPIC, rclcpp::QoS(1),
            std::bind(&OccupancyGrid::clustersCallback, this, std::placeholders::_1));
        
        localization_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            POSE_TOPIC, rclcpp::QoS(1),
            std::bind(&OccupancyGrid::localization_callback, this, std::placeholders::_1));
        
        path_sub = this->create_subscription<sac_interfaces::msg::PathIds>(
            PATH_TOPIC, 10, 
            std::bind(&OccupancyGrid::pathCallback, this, std::placeholders::_1));

        // Inform about node initialization.
        RCLCPP_INFO(this->get_logger(), 
                    "Grid_resolution: %.2f and grid_size: %d", 
                    this->grid_resolution_, this->grid_size_);
    }

    void OccupancyGrid::clustersCallback(const cluster_msgs::msg::ClusterPointsArray & msg){
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        occupancy_grid.header.stamp = msg.header.stamp;
        occupancy_grid.header.frame_id = "map";
        occupancy_grid.info.resolution = this->grid_resolution_;
        occupancy_grid.info.width = this->grid_size_;
        occupancy_grid.info.height = this->grid_size_;

        int start_x = this->grid_size_ / 2;
        int start_y = this->grid_size_ / 2;
        double yaw = tf2::getYaw(this->pose.orientation);
        
        double offset = (this->grid_size_ * this->grid_resolution_) / 2.0;
        double dx = -offset * std::cos(yaw) + offset * std::sin(yaw);
        double dy = -offset * std::sin(yaw) - offset * std::cos(yaw);

        occupancy_grid.info.origin.position.x = this->pose.position.x + dx;
        occupancy_grid.info.origin.position.y = this->pose.position.y + dy;
        occupancy_grid.info.origin.orientation = this->pose.orientation;

        std::vector<int8_t> grid_data(this->grid_size_ * this->grid_size_, 100);

        // ----- LANELET PATH MASK -----
        cv::Mat lanelet_mask = cv::Mat::ones(this->grid_size_, this->grid_size_, CV_8U) * 100;

        if (this->lanelet_map_ == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Lanelet map is not loaded!");
            return;
        }
        if (this->path_.lanes.empty())
        {
            return;
        }

        // Resolve this once per cloud. Repeating a failed TF lookup and warning for
        // every cluster point created an unbounded replay log storm and wasted the
        // planning executor while the replay graph had no map<-lidar transform.
        geometry_msgs::msg::TransformStamped cluster_transform;
        try
        {
            cluster_transform = tf_buffer_.lookupTransform(
                occupancy_grid.header.frame_id, msg.header.frame_id,
                rclcpp::Time(msg.header.stamp));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Cluster transform unavailable; occupancy output skipped: %s", ex.what());
            return;
        }

        // Create a marker array for lanelet visualization
        visualization_msgs::msg::MarkerArray lanelet_markers;

        visualization_msgs::msg::Marker lanelet_marker_left;
        lanelet_marker_left.header.frame_id = occupancy_grid.header.frame_id;
        lanelet_marker_left.header.stamp = this->get_clock()->now();
        lanelet_marker_left.ns = "lanelet_markers_left";
        lanelet_marker_left.id = 1;
        lanelet_marker_left.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lanelet_marker_left.action = visualization_msgs::msg::Marker::ADD;
        lanelet_marker_left.scale.x = 0.1; // Line width
        lanelet_marker_left.color.r = 1.0f;
        lanelet_marker_left.color.g = 0.0f;
        lanelet_marker_left.color.b = 0.0f;
        lanelet_marker_left.color.a = 1.0f;

        visualization_msgs::msg::Marker lanelet_marker_right;
        lanelet_marker_right.header.frame_id = occupancy_grid.header.frame_id;
        lanelet_marker_right.header.stamp = this->get_clock()->now();
        lanelet_marker_right.ns = "lanelet_markers_right";
        lanelet_marker_right.id = 2;
        lanelet_marker_right.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lanelet_marker_right.action = visualization_msgs::msg::Marker::ADD;
        lanelet_marker_right.scale.x = 0.1; // Line width
        lanelet_marker_right.color.r = 0.0f;
        lanelet_marker_right.color.g = 0.0f;
        lanelet_marker_right.color.b = 1.0f;
        lanelet_marker_right.color.a = 1.0f; 

        std::vector<cv::Point> contour_points;
        std::vector<lanelet::BasicPoint2d> centerline_points;

        sac_interfaces::msg::PathIds path_ids;
        
        for (lanelet::ConstLanelet lane : this->lanelet_map_->laneletLayer) {
        //   RCLCPP_INFO(this->get_logger(), "%lu. lane id: %ld", i++, lane.id());
          path_ids.lanes.push_back(lane.id());
          for (const auto& point : lane.centerline2d()) {
            centerline_points.push_back(lanelet::BasicPoint2d(point.x(), point.y()));
          }
        }

        for (uint16_t lanelet_id : path_ids.lanes) {
            if (!this->lanelet_map_->laneletLayer.exists(lanelet::Id(lanelet_id))) continue;
            const auto& lanelet = this->lanelet_map_->laneletLayer.get(lanelet::Id(lanelet_id));
        
            contour_points.clear();
        
            auto transform_point = [&](const lanelet::ConstPoint3d& pt) -> std::optional<cv::Point> {
                double dx = pt.x() - occupancy_grid.info.origin.position.x;
                double dy = pt.y() - occupancy_grid.info.origin.position.y;
                double c = std::cos(-yaw);
                double s = std::sin(-yaw);
                double rot_x = dx * c - dy * s;
                double rot_y = dx * s + dy * c;
                int x = static_cast<int>(rot_x / this->grid_resolution_);
                int y = static_cast<int>(rot_y / this->grid_resolution_);
                return cv::Point(x, y);
            };
        
            // Append left bound points
            for (const auto& pt : lanelet.leftBound()) {
                auto opt_point = transform_point(pt);
                if (opt_point) {
                    contour_points.push_back(*opt_point);
                    if (this->debug_mode) {
                        geometry_msgs::msg::PointStamped in_point;
                        in_point.header.frame_id = "map";
                        in_point.point.x = pt.x();
                        in_point.point.y = pt.y();
                        in_point.point.z = 0.0;
                        lanelet_marker_left.points.push_back(in_point.point);
                    }
                }
            }
        
            // Append right bound points in reverse
            const auto& rb = lanelet.rightBound();
            for (int i = static_cast<int>(rb.size()) - 1; i >= 0; --i) {
                const auto& pt = rb[i];
                auto opt_point = transform_point(pt);
                if (opt_point) {
                    contour_points.push_back(*opt_point);
            
                    if (this->debug_mode) {
                        geometry_msgs::msg::PointStamped in_point;
                        in_point.header.frame_id = "map";
                        in_point.point.x = pt.x();
                        in_point.point.y = pt.y();
                        in_point.point.z = 0.0;
                        lanelet_marker_right.points.push_back(in_point.point);
                    }
                }
            }
        
            if (contour_points.size() < 3) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Too few points for polygon fill, skipping.");
                continue;
            }
        
            // Convert to Clipper2 polygon
            std::vector<Clipper2Lib::PointD> subject_path;
            for (const auto& pt : contour_points) {
                subject_path.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
            }
        
            // Define the grid boundary for clipping
            std::vector<Clipper2Lib::PointD> clip_rect = {
                {0.0, 0.0},
                {static_cast<double>(lanelet_mask.cols - 1), 0.0},
                {static_cast<double>(lanelet_mask.cols - 1), static_cast<double>(lanelet_mask.rows - 1)},
                {0.0, static_cast<double>(lanelet_mask.rows - 1)}
            };
        
            Clipper2Lib::ClipperD clipper;
            clipper.AddSubject({subject_path});
            clipper.AddClip({clip_rect});
            std::vector<std::vector<Clipper2Lib::PointD>> solution;
            clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, solution);
        
            // Fill the clipped polygon
            for (const auto& poly : solution) {
                std::vector<cv::Point> poly_cv;
                for (const auto& pt : poly) {
                    poly_cv.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
                }
                std::vector<std::vector<cv::Point>> fill_contour = {poly_cv};
                cv::fillPoly(lanelet_mask, fill_contour, cv::Scalar(0)); // 0 = free
            }
        }

        if(this->debug_mode){
        // Publish lanelet markers
        lanelet_markers.markers.push_back(lanelet_marker_left);
        lanelet_markers.markers.push_back(lanelet_marker_right);
        lanelet_marker_pub_->publish(lanelet_markers);
        }

        // ----- DYNAMIC CLUSTERS MASK -----
        cv::Mat obstacle_mask = cv::Mat::zeros(this->grid_size_, this->grid_size_, CV_8U);
        for (const auto &cluster : msg.clusters)
        {
            std::vector<cv::Point> cluster_points;
            for (const auto &point : cluster.bottom_points_array)
            {
                // point is in the lidar frame 
                geometry_msgs::msg::PointStamped cluster_point;
                cluster_point.header.frame_id = msg.header.frame_id; // actual lidar frame
                cluster_point.point.x = point.x;
                cluster_point.point.y = point.y;
                cluster_point.point.z = point.z;

                // Transform the point to the occupancy grid frame
                geometry_msgs::msg::PointStamped transformed_point;
                tf2::doTransform(cluster_point, transformed_point, cluster_transform);

                double dx = transformed_point.point.x - occupancy_grid.info.origin.position.x;
                double dy = transformed_point.point.y - occupancy_grid.info.origin.position.y;

                double rot_x =  dx * std::cos(-yaw) - dy * std::sin(-yaw);
                double rot_y =  dx * std::sin(-yaw) + dy * std::cos(-yaw);

                int x = rot_x / this->grid_resolution_;
                int y = rot_y / this->grid_resolution_;

                if (x >= 0 && x < this->grid_size_ && y >= 0 && y < this->grid_size_)
                {
                    cluster_points.emplace_back(x, y);
                }
            }

            if (!cluster_points.empty())
            {
                std::vector<cv::Point> hull;
                cv::convexHull(cluster_points, hull);
                std::vector<std::vector<cv::Point>> hull_contour = {hull};
                cv::fillPoly(obstacle_mask, hull_contour, cv::Scalar(255));
            }
        }

        // Effective inflation radius: half the diagonal of vehicle + margin
        double inflation_radius_m = std::sqrt(std::pow(this->vehicle_width / 2.0, 2) +
        std::pow(this->vehicle_length / 2.0, 2)) + this->safety_margin;

        // Convert to pixels
        int inflation_radius_px = static_cast<int>(std::ceil(inflation_radius_m / this->grid_resolution_));

        // Create circular kernel
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                            cv::Size(2 * inflation_radius_px + 1, 2 * inflation_radius_px + 1),
                            cv::Point(inflation_radius_px, inflation_radius_px));

        // Apply dilation
        cv::dilate(obstacle_mask, obstacle_mask, kernel);

        // ----- COMBINE AND UPDATE LOG ODDS -----
        for (int y = 0; y < this->grid_size_; ++y)
        {
            for (int x = 0; x < this->grid_size_; ++x)
            {
                int idx = y * this->grid_size_ + x;

                bool is_obstacle     = (obstacle_mask.at<uint8_t>(y, x) > 0);
                bool is_lanelet_free = (lanelet_mask.at<uint8_t>(y, x) == 0);

                if (is_obstacle){
                    grid_data[idx] = 100;
                }
                else if (is_lanelet_free){
                    grid_data[idx] = 0;
                }
                else{ // unknown which is probably outside of the lane but we already cover it in the trajectory planner.
                    grid_data[idx] = 50;
                }
            }
        }

        // Start cell visualization
        grid_data[start_y * this->grid_size_ + start_x] = 31;

        occupancy_grid.data = grid_data;
        occupancy_grid_pub_->publish(occupancy_grid);
        occupancy_ready = true;
    }

    void OccupancyGrid::localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
    {
        this->pose = msg.pose.pose;
    }

    void OccupancyGrid::pathCallback(const sac_interfaces::msg::PathIds & msg){
        this->path_ = msg;
    }

    std::optional<cv::Point> OccupancyGrid::transform_point(const lanelet::ConstPoint3d& pt) const {
        double yaw = tf2::getYaw(this->pose.orientation);
    
        double dx = pt.x() - this->pose.position.x;
        double dy = pt.y() - this->pose.position.y;
    
        double c = std::cos(-yaw);
        double s = std::sin(-yaw);
    
        double rot_x = dx * c - dy * s;
        double rot_y = dx * s + dy * c;
    
        int x = static_cast<int>(rot_x / this->grid_resolution_) + this->grid_size_ / 2;
        int y = static_cast<int>(rot_y / this->grid_resolution_) + this->grid_size_ / 2;
    
        if (x >= 0 && x < this->grid_size_ && y >= 0 && y < this->grid_size_) {
            return cv::Point(x, y);
        } else {
            return std::nullopt;
        }
    }

    void OccupancyGrid::readLaneletMap()
    {
        // Create a projector using your origin
        lanelet::Origin origin({this->origin_x, this->origin_y, 0.0});

        lanelet::projection::UtmProjector projector = lanelet::projection::UtmProjector(origin);
        
        this->lanelet_map_ = lanelet::load(this->osm_path, projector);

        if (this->lanelet_map_ == nullptr)
        {
        RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet map");
        rclcpp::shutdown(); // BE CAREFUL FOR CONTAINERIZED NODES
        }
        else{
        RCLCPP_INFO(this->get_logger(), "Lanelet map is loaded successfully!");
        }
    }

} // namespace smart_car

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::OccupancyGrid)
