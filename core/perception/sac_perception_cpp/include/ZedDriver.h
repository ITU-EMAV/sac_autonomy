
#ifndef __CAMERA_ZED_
#define __CAMERA_ZED_

#include <torch/torch.h>
#include <torch/script.h>

// ZED includes
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#include "ClientPublisher.hpp"

// Ros2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
//std
#include <mutex>

using namespace rclcpp;

namespace sensors
{

    class ZedDriver :public Node
    {
        public:

        

        ZedDriver(const NodeOptions &,std::string);

        std::vector<ClientPublisher *> clients;
        std::mutex mtx;
        private:
        

      

    };
}
#endif