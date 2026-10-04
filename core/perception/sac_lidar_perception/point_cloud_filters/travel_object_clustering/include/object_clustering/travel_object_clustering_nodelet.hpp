#ifndef TRAVEL_OBJECT_CLUSTERING_NODELET_HPP
#define TRAVEL_OBJECT_CLUSTERING_NODELET_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <list>
#include <numeric>
#include <random>
#include <chrono>
#include <forward_list>
#include <boost/optional.hpp>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include <cluster_msgs/msg/cluster_points_array.hpp>
#include <cluster_msgs/msg/cluster_points.hpp>

namespace object_clustering {

  using PointType = pcl::PointXYZI;

  struct AOSNode {
    uint start;
    uint end;
    int label = -1;
  };

  struct Point {
    float x, y, z;
    uint idx;
    bool valid = false;
    int label;
  };

  class TravelObjectClusteringComponent : public rclcpp::Node {
  private:
    uint16_t max_label_{};
    std::vector<std::forward_list<Point*>> clusters_;
    std::forward_list<Point*> empty_;
    std::vector<int> valid_cnt_;
    std::vector<std::vector<AOSNode>> nodes_;
    std::vector<float> vert_angles_;
    std::vector<std::vector<Point>> range_mat_;
    std::vector<std::vector<Point>> emptyrange_mat_;

    float MIN_RANGE;
    float MAX_RANGE;
    float FRONT_RANGE;
    float REAR_RANGE;
    float LATERAL_RANGE;
    std::string FRAME_ID;
    std::string INPUT_TOPIC;
    int VERT_SCAN;
    int HORZ_SCAN;
    float CAR_TOP_MARGIN;
    float CAR_LAT_MARGIN;
    float CAR_FRONT_MARGIN;
    float VERT_MERGE_THRES;
    float HORZ_MERGE_THRES;
    int VERT_SCAN_SIZE;
    int HORZ_SCAN_SIZE;
    int HORZ_SKIP_SIZE;
    int HORZ_EXTENSION_SIZE;
    int DOWNSAMPLE;
    float MIN_VERT_ANGLE;
    float MAX_VERT_ANGLE;
    boost::optional<int> MIN_CLUSTER_SIZE;
    boost::optional<int> MAX_CLUSTER_SIZE;
    float MIN_CLUSTER_HEIGHT;
    float MAX_BOTTOM_Z;
    /// Points above this height in the sensor frame are dropped before clustering.
    float MAX_POINT_Z;
    float resolution;
    bool can_see_back;
    uint marker_id{};
    std::shared_ptr<visualization_msgs::msg::MarkerArray> marker_array_;
    builtin_interfaces::msg::Time stamp_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_labeled_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_polygon_publisher_;
    rclcpp::Publisher<cluster_msgs::msg::ClusterPointsArray>::SharedPtr cluster_bottom_arr_publisher_;

    void show_parameters() {
      RCLCPP_INFO(this->get_logger(), "max_range: %f", MAX_RANGE);
      RCLCPP_INFO(this->get_logger(), "min_range: %f", MIN_RANGE);
      RCLCPP_INFO(this->get_logger(), "front_range: %f", FRONT_RANGE);
      RCLCPP_INFO(this->get_logger(), "rear_range: %f", REAR_RANGE);
      RCLCPP_INFO(this->get_logger(), "lateral_range: %f", LATERAL_RANGE);
      RCLCPP_INFO(this->get_logger(), "frame_id: %s", FRAME_ID.c_str());
      RCLCPP_INFO(this->get_logger(), "input_topic: %s", INPUT_TOPIC.c_str());
      RCLCPP_INFO(this->get_logger(), "vert_scan: %d", VERT_SCAN);
      RCLCPP_INFO(this->get_logger(), "horz_scan: %d", HORZ_SCAN);
      RCLCPP_INFO(this->get_logger(), "car_top_margin: %f", CAR_TOP_MARGIN);
      RCLCPP_INFO(this->get_logger(), "car_lat_margin: %f", CAR_LAT_MARGIN);
      RCLCPP_INFO(this->get_logger(), "car_front_margin: %f", CAR_FRONT_MARGIN);
      RCLCPP_INFO(this->get_logger(), "vert_merge_thres: %f", VERT_MERGE_THRES);
      RCLCPP_INFO(this->get_logger(), "horz_merge_thres: %f", HORZ_MERGE_THRES);
      RCLCPP_INFO(this->get_logger(), "vert_scan_size: %d", VERT_SCAN_SIZE);
      RCLCPP_INFO(this->get_logger(), "horz_scan_size: %d", HORZ_SCAN_SIZE);
      RCLCPP_INFO(this->get_logger(), "horz_skip_size: %d", HORZ_SKIP_SIZE);
      RCLCPP_INFO(this->get_logger(), "horz_extension_size: %d", HORZ_EXTENSION_SIZE);
      RCLCPP_INFO(this->get_logger(), "downsample: %d", DOWNSAMPLE);
      RCLCPP_INFO(this->get_logger(), "min_vert_angle: %f", MIN_VERT_ANGLE);
      RCLCPP_INFO(this->get_logger(), "max_vert_angle: %f", MAX_VERT_ANGLE);
      RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", MIN_CLUSTER_SIZE.value_or(-1));
      RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", MAX_CLUSTER_SIZE.value_or(-1));
      RCLCPP_INFO(this->get_logger(), "vert_resolution: %f", resolution);
    };

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

    void segmentObjects(pcl::PointCloud<PointType>::Ptr cloud_in,
                        pcl::PointCloud<PointType>::Ptr cloud_out);

    void sphericalProjection(pcl::PointCloud<PointType>::Ptr cloud_in);

    void labelPointcloud(pcl::PointCloud<PointType>::Ptr cloud_in,
                         pcl::PointCloud<PointType>::Ptr cloud_out);

    void horizontalUpdate(int channel);

    void verticalUpdate(int channel);

    bool overlap(AOSNode &first_node, AOSNode &second_node) const;

    bool mergeNodes(AOSNode &first_node, AOSNode &second_node, int cur_channel, int prev_channel, int query_idx);

    float pointDistance(Point pt1, Point pt2) {
      return sqrt((pt1.x - pt2.x)*(pt1.x-pt2.x) + (pt1.y-pt2.y)*(pt1.y-pt2.y));
    };

    bool nodeDistance(AOSNode &first_node, AOSNode &second_node, Point &first_point, Point &second_point);

    static float getRange(PointType pt);

    int getRowIdx(PointType pt);

    [[maybe_unused]] int getRowIdx(PointType pt, std::vector<float> vert_angles);

    int getColIdx(PointType pt) const;

    void mergeClusters(uint16_t cur_label, uint16_t target_label);

  public:
    explicit TravelObjectClusteringComponent(const rclcpp::NodeOptions& options);
  };

} // namespace object_clustering

#endif // TRAVEL_OBJECT_CLUSTERING_NODELET_HPP
