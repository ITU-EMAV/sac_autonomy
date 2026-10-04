#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "Planner.h"
#include "A_star.h"
using namespace std;
using std::placeholders::_1;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    global_frame = this->declare_parameter<std::string>("global_frame", "odom");
    local_frame = this->declare_parameter<std::string>("local_frame", "base_link");

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // callback group
    my_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = my_callback_group;

    // timer
    // timer_ptr_ = this->create_wall_timer(1000ms, bind(&NavigationNode::timer_cb, this), my_callback_group);

    // subscribers
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&NavigationNode::goal_pose_cb, this, _1), options);
    og_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", 10, std::bind(&NavigationNode::og_cb, this, _1), options);

    // publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  void timer_cb()
  {
    RCLCPP_INFO(this->get_logger(), "timer cb");
    // start a timer to measure the time
    auto start_time = std::chrono::high_resolution_clock::now();

    if (this->og == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "There is no occupancy grid");
      return;
    }
    if (this->goal_pose_global == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "There is no goal pose");
      return;
    }

    if (goal_pose_global->header.frame_id != local_frame)
    {
      RCLCPP_INFO(this->get_logger(), "Transforming goal pose to local frame");
      // transform the goal pose to the local frame
      geometry_msgs::msg::TransformStamped transform_stamped;
      try
      {
        transform_stamped = tf_buffer_->lookupTransform(local_frame, goal_pose_global->header.frame_id, tf2::TimePointZero);
        // *goal_pose = tf_buffer_->transform(*goal_pose, local_frame);
        tf2::doTransform(*goal_pose_global, goal_pose, transform_stamped);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
        return;
      }
    }
    // check if the goal pose is in the map
    if (goal_pose.pose.position.x < this->og->info.origin.position.x ||
        goal_pose.pose.position.y < this->og->info.origin.position.y ||
        goal_pose.pose.position.x > this->og->info.origin.position.x + this->og->info.width * this->og->info.resolution ||
        goal_pose.pose.position.y > this->og->info.origin.position.y + this->og->info.height * this->og->info.resolution)
    {
      RCLCPP_WARN(this->get_logger(), "Goal pose is out of the map");
      this->goal_pose_global = nullptr; // do not plan
      return;
    }

    // coppy og->data to a 2d array with memcpy
    int8_t arr[this->og->info.height][this->og->info.width];
    memcpy(arr, this->og->data.data(), this->og->info.height * this->og->info.width * sizeof(int8_t));

    // (int8_t *) this->og->data.data()
    // cout << this->og->info.height << this->og->info.height << this->og->info.resolution << this->og->info.origin.position.x << this->og->info.origin.position.y << endl;
    a_star.update_og((int8_t *)arr, this->og->info.height, this->og->info.width, this->og->info.resolution, this->og->info.origin.position.x, this->og->info.origin.position.y);
    // RCLCPP_WARN(this->get_logger(), "update_og");

    float goal_x = this->goal_pose.pose.position.x;
    float goal_y = this->goal_pose.pose.position.y;
    float goal_z = this->goal_pose.pose.position.z;

    // cout << goal_x << " " << goal_y << " " << goal_z << endl;

    node_2d start = node_2d((float)0.0, (float)0.0, (float)0.0); // TODO get the pose from tf

    node_2d end = node_2d(goal_x, goal_y, goal_z);
    // cout << end.x.fdata << endl;

    node_2d *ptr = a_star.plan(start, end);

    // calculate the time from start and print it
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Time: %d ms", duration.count());
    // this->runner = thread(&NavigationNode::pub_path_from_node_2d, [this, ptr]() {});

    if (ptr == nullptr)
    {
      a_star.clear_garbage();
      RCLCPP_ERROR(this->get_logger(), "Path not found.");
      return;
    }
    pub_path_from_node_2d(ptr);
    a_star.clear_garbage();
  }
  void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // delete this->goal_pose;
    this->goal_pose_global = msg;

    // if (goal_pose_global->header.frame_id != local_frame)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Transforming goal pose to local frame");
    //   // transform the goal pose to the local frame
    //   geometry_msgs::msg::TransformStamped transform_stamped;
    //   try
    //   {
    //     transform_stamped = tf_buffer_->lookupTransform(local_frame, goal_pose_global->header.frame_id, tf2::TimePointZero);
    //     // *goal_pose = tf_buffer_->transform(*goal_pose, local_frame);
    //     tf2::doTransform(*goal_pose_global, *goal_pose, transform_stamped);
    //   }
    //   catch (tf2::TransformException &ex)
    //   {
    //     RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
    //     return;
    //   }
    // }

    // // check if the occupancy grid is available
    // if (this->og == nullptr)
    // {
    //   RCLCPP_WARN(this->get_logger(), "There is no occupancy grid");
    //   this->goal_pose_global = nullptr; // do not plan
    //   return;
    // }

    // // check if the goal pose is in the map
    // if (goal_pose->pose.position.x < this->og->info.origin.position.x ||
    //     goal_pose->pose.position.y < this->og->info.origin.position.y ||
    //     goal_pose->pose.position.x > this->og->info.origin.position.x + this->og->info.width * this->og->info.resolution ||
    //     goal_pose->pose.position.y > this->og->info.origin.position.y + this->og->info.height * this->og->info.resolution)
    // {
    //   RCLCPP_WARN(this->get_logger(), "Goal pose is out of the map");
    //   this->goal_pose_global = nullptr; // do not plan
    //   return;
    // }

    // delete this->goal_pose;
    // this->goal_pose = new geometry_msgs::msg::PoseStamped(*msg);
    // RCLCPP_INFO(this->get_logger(), "Received message");
  }
  void og_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // delete this->og;
    this->og = msg;
    timer_cb();
    // RCLCPP_INFO(this->get_logger(), "Received message");
  }

  void pub_path_from_node_2d(node_2d *ptr)
  {
    nav_msgs::msg::Path path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = local_frame;
    path_msg.header.stamp = this->now();

    while (ptr->parent != nullptr)
    {
      // cout << ptr->x.fdata << " " << ptr->y.fdata << " " << ptr->cost.g_cost << endl;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = local_frame;
      pose.header.stamp = this->now();
      pose.pose.position.x = ptr->x.fdata;
      pose.pose.position.y = ptr->y.fdata;

      path_msg.poses.push_back(pose);
      ptr = ptr->parent;
    }
    // transform path to global frame
    RCLCPP_INFO(this->get_logger(), "Transforming path to global frame");
    path_msg.header.frame_id = global_frame;

    for (auto &pose : path_msg.poses)
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      try
      {
        transform_stamped = tf_buffer_->lookupTransform(global_frame, local_frame, tf2::TimePointZero);
        tf2::doTransform(pose, pose, transform_stamped);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
        return;
      }
    }
    // reverse the path
    std::reverse(path_msg.poses.begin(), path_msg.poses.end());
    this->path_pub_->publish(path_msg);
  }

  // variables
  A_star a_star = A_star(0, 0.05);
  geometry_msgs::msg::PoseStamped goal_pose;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_global = nullptr;

  // nav_msgs::msg::OccupancyGrid *og = nullptr;
  nav_msgs::msg::OccupancyGrid::SharedPtr og = nullptr;

  // timer
  rclcpp::TimerBase::SharedPtr timer_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr og_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::CallbackGroup::SharedPtr my_callback_group;
  rclcpp::SubscriptionOptions options;

  // tf
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame;
  std::string local_frame;

  std::thread runner;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;

  // node_2d start = node_2d((float)0.0, (float)0.0, (float)0.0);
  // node_2d end = node_2d((float)-30.0, (float)30.0, (float)0.0);

  // node_2d *ptr = a_star.plan(start, end);
  // cout << "solution" << endl;
  // while (ptr->parent != nullptr)
  // {
  //   cout << ptr->x.fdata << " " << ptr->y.fdata << " " << ptr->cost.g_cost << endl;
  //   ptr = ptr->parent;
  // }

  // priority_queue<node_2d *> pq;

  // node_2d *a = new node_2d((float)0.0, (float)0.0, (float)0.0, 1, 1);
  // node_2d *b = new node_2d((float)0.0, (float)0.0, (float)0.0, 2, 3);

  // pq.push(a);
  // pq.push(b);
  // auto c = pq.top();
  // cout << c->cost.h_cost << endl;

  // for(int i = 0; i < 10 ; i++)
  // {

  //   for(int j = 0 ; j < 10 ; j++)
  //    {
  //     arr[i][j] = i+j;
  //     cout << i +j  << " ";
  //    }
  //    cout << endl;

  // }
  // cout << endl;
  // a_star.update_og((float*)arr,10,10,0.05);

  //  for(int i = 0; i < 10 ; i++)
  // {
  //   for(int j = 0 ; j < 10 ; j++)
  //   {
  //     a_star.get_mark(i,j)  = 1;
  //   }

  // }

  // for(int i = 0; i < 10 ; i++)
  // {
  //   for(int j = 0 ; j < 10 ; j++)
  //   {
  //     cout << a_star.get_mark(i,j)  << " ";
  //   }
  //   cout << endl;
  // }
}
