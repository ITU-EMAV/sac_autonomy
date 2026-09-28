#include "sac_localization/ros/ros_params.hpp"

#include <stdexcept>
#include <utility>

namespace sac_localization
{

RosParams::RosParams(rclcpp::Node * node, std::string prefix)
: node_(node), prefix_(std::move(prefix)) {}

bool RosParams::has(const std::string & key) const
{
  return node_->has_parameter(prefix_ + key) &&
         node_->get_parameter(prefix_ + key).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

rclcpp::Parameter RosParams::get(const std::string & key) const
{
  return node_->get_parameter(prefix_ + key);
}

void RosParams::wrongType(const std::string & key, const std::string & expected) const
{
  throw std::invalid_argument("Parameter " + prefix_ + key + " must be " + expected);
}

double RosParams::getDouble(const std::string & key, double fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return p.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(p.as_int());
    default:
      wrongType(key, "a number");
  }
}

int RosParams::getInt(const std::string & key, int fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    wrongType(key, "a whole number");
  }
  return static_cast<int>(p.as_int());
}

bool RosParams::getBool(const std::string & key, bool fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    wrongType(key, "true or false");
  }
  return p.as_bool();
}

std::string RosParams::getString(const std::string & key, const std::string & fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    wrongType(key, "a string");
  }
  return p.as_string();
}

std::vector<double> RosParams::getDoubles(const std::string & key, const std::vector<double> & fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return p.as_double_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      std::vector<double> values;
      for (auto v : p.as_integer_array()) {
        values.push_back(static_cast<double>(v));
      }
      return values;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return {p.as_double()};
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return {static_cast<double>(p.as_int())};
    default:
      wrongType(key, "a list of numbers");
  }
}

std::vector<std::string> RosParams::getStrings(
  const std::string & key, const std::vector<std::string> & fallback) const
{
  if (!has(key)) {
    return fallback;
  }
  const rclcpp::Parameter p = get(key);
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return p.as_string_array();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return {p.as_string()};
    default:
      wrongType(key, "a list of strings");
  }
}

}  // namespace sac_localization
