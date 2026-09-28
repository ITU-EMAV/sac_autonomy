// Params over a node's parameters (the node allows undeclared parameters and declares those
// in its YAML). Whole numbers are accepted where a floating point value is expected.

#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sac_perception/params.hpp"

namespace sac_perception
{

class RosParams : public Params
{
public:
  RosParams(rclcpp::Node * node, std::string prefix) : node_(node), prefix_(std::move(prefix)) {}

  const std::string & prefix() const { return prefix_; }

  bool has(const std::string & key) const override
  {
    return node_->has_parameter(prefix_ + key) &&
           node_->get_parameter(prefix_ + key).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  }

  double getDouble(const std::string & key, double fallback) const override
  {
    if (!has(key)) {
      return fallback;
    }
    const rclcpp::Parameter p = node_->get_parameter(prefix_ + key);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return p.as_double();
    }
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }
    throw std::invalid_argument("Parameter " + prefix_ + key + " must be a number");
  }

  bool getBool(const std::string & key, bool fallback) const override
  {
    if (!has(key)) {
      return fallback;
    }
    const rclcpp::Parameter p = node_->get_parameter(prefix_ + key);
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      throw std::invalid_argument("Parameter " + prefix_ + key + " must be true or false");
    }
    return p.as_bool();
  }

  std::string getString(const std::string & key, const std::string & fallback) const override
  {
    if (!has(key)) {
      return fallback;
    }
    const rclcpp::Parameter p = node_->get_parameter(prefix_ + key);
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      throw std::invalid_argument("Parameter " + prefix_ + key + " must be a string");
    }
    return p.as_string();
  }

  std::vector<double> getDoubles(const std::string & key, const std::vector<double> & fallback) const override
  {
    if (!has(key)) {
      return fallback;
    }
    const rclcpp::Parameter p = node_->get_parameter(prefix_ + key);
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
        throw std::invalid_argument("Parameter " + prefix_ + key + " must be a list of numbers");
    }
  }

  std::vector<std::string> getStrings(
    const std::string & key, const std::vector<std::string> & fallback) const override
  {
    if (!has(key)) {
      return fallback;
    }
    const rclcpp::Parameter p = node_->get_parameter(prefix_ + key);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      return p.as_string_array();
    }
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return {p.as_string()};
    }
    throw std::invalid_argument("Parameter " + prefix_ + key + " must be a list of strings");
  }

private:
  rclcpp::Node * node_;
  std::string prefix_;
};

}  // namespace sac_perception
