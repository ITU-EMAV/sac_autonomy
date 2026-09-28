// Params over the node's ROS parameters, under a prefix ("estimator.", "sensors.<name>.").
// The node accepts undeclared parameters and declares everything in its YAML, so values are
// read (and can be changed with `ros2 param set`) without declaring each key. Whole numbers
// are accepted where a floating point value is expected.

#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sac_localization/core/params.hpp"

namespace sac_localization
{

class RosParams : public Params
{
public:
  RosParams(rclcpp::Node * node, std::string prefix);

  bool has(const std::string & key) const override;
  double getDouble(const std::string & key, double fallback) const override;
  int getInt(const std::string & key, int fallback) const override;
  bool getBool(const std::string & key, bool fallback) const override;
  std::string getString(const std::string & key, const std::string & fallback) const override;
  std::vector<double> getDoubles(const std::string & key, const std::vector<double> & fallback) const override;
  std::vector<std::string> getStrings(
    const std::string & key, const std::vector<std::string> & fallback) const override;

  const std::string & prefix() const { return prefix_; }

private:
  rclcpp::Parameter get(const std::string & key) const;
  [[noreturn]] void wrongType(const std::string & key, const std::string & expected) const;

  rclcpp::Node * node_;
  std::string prefix_;
};

}  // namespace sac_localization
