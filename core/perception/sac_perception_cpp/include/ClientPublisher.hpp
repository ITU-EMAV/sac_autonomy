#ifndef  __SENDER_RUNNER_HDR__
#define __SENDER_RUNNER_HDR__

#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>



#include <thread>

#include <torch/torch.h>
#include <torch/script.h>

//std
#include <mutex>
//nvtx
#include <nvtx3/nvToolsExt.h>

using namespace torch::indexing;
using namespace std;
namespace F = torch::nn::functional;


class ClientPublisher{

public:
    ClientPublisher(rclcpp::Node & node,int id,std::mutex &);
    ~ClientPublisher();

    bool open(sl::FusionConfiguration);
    void start();
    void stop();

    bool isRunning() {
        return running;
    }
    //utils
    void to_tensor(sl::Mat & point_cloud , torch::Tensor & tensor);
    void flatten_tensor(torch::Tensor & tensor);

    void visulize_marker(bool);
    void publish_laser_scan(bool);
    void publish_pointcloud(bool);
    void publish_pose(bool);
    void publish_odom(bool);
    void publish_imu(bool);

    


    void deleter(void* );
    
    //publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


    //ros msgs
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    nav_msgs::msg::Odometry odom_msg;
    sensor_msgs::msg::LaserScan laser_msg;
    visualization_msgs::msg::Marker marker;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    sensor_msgs::msg::Imu imu_msg;
    
    rclcpp::Node* node_;

    

private:
    char memory_string[50];

    sl::Camera zed;
    sl::Translation translations;
    sl::Pose pose;
    sl::SensorsData sensors_data;
    sl::Timestamp last_imu_ts = 0;

    void work();
    void publish_sensors();

    float voxel_size = 5; //cm

    bool publish_marker_open = false;
    bool publish_point_cloud_open = true;
    bool publish_laser_scan_open = false;
    bool publish_imu_open = true;
    bool publish_odom_open = false;
    bool publish_pose_open = false;
    bool position_tracking_open = false;

    
    bool running;
    int serial;
    int ind_len;
    int id_;
    int64_t pointcloud_timestamp_ns;
    int64_t pointcloud_timestamp_s;
    

    std::string base_frame;

    std::mutex * mtx_;
    std::mutex mtx_msg;
    std::thread runner;
    std::thread sensors_runner;



    torch::Tensor tensor;
    torch::Tensor cpu_tensor;
    torch::Tensor ranges_tensor_cpu;

    torch::Tensor rot_mat;
    torch::Tensor trans_mat;

    struct TimestampHandler {

        // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
        inline bool isNew(sl::Timestamp& ts_curr, sl::Timestamp& ts_ref) {
            bool new_ = ts_curr > ts_ref;
            if (new_) ts_ref = ts_curr;
            return new_;
        }
        // Specific function for IMUData.
        inline bool isNew(sl::SensorsData::IMUData& imu_data) {
            return isNew(imu_data.timestamp, ts_imu);
        }
        // Specific function for MagnetometerData.
        inline bool isNew(sl::SensorsData::MagnetometerData& mag_data) {
            return isNew(mag_data.timestamp, ts_mag);
        }
        // Specific function for BarometerData.
        inline bool isNew(sl::SensorsData::BarometerData& baro_data) {
            return isNew(baro_data.timestamp, ts_baro);
        }

        sl::Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
    } ts;

};

#endif // ! __SENDER_RUNNER_HDR__