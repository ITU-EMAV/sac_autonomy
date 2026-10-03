// Parameters of a plugin, without ROS: the node implements this over ROS parameters (with
// the plugin's prefix, e.g. "estimator." or "sensors.middle_imu."), tests over a map.

#pragma once

#include <string>
#include <vector>

namespace sac_localization
{

class Params
{
public:
  virtual ~Params() = default;

  virtual bool has(const std::string & key) const = 0;
  virtual double getDouble(const std::string & key, double fallback) const = 0;
  virtual int getInt(const std::string & key, int fallback) const = 0;
  virtual bool getBool(const std::string & key, bool fallback) const = 0;
  virtual std::string getString(const std::string & key, const std::string & fallback) const = 0;
  virtual std::vector<double> getDoubles(
    const std::string & key, const std::vector<double> & fallback) const = 0;
  virtual std::vector<std::string> getStrings(
    const std::string & key, const std::vector<std::string> & fallback) const = 0;
};

}  // namespace sac_localization
