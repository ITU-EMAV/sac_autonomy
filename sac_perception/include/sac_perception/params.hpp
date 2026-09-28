// Parameters of a plugin: the node reads them from its ROS parameters under a prefix
// ("sources.roof_lidar.", "sources.roof_lidar.ground."; ros_params.hpp), tests from a map.
// Same idea as sac_localization's Params; kept here so that the packages stay independent.

#pragma once

#include <map>
#include <string>
#include <vector>

namespace sac_perception
{

class Params
{
public:
  virtual ~Params() = default;
  virtual bool has(const std::string & key) const = 0;
  virtual double getDouble(const std::string & key, double fallback) const = 0;
  virtual bool getBool(const std::string & key, bool fallback) const = 0;
  virtual std::string getString(const std::string & key, const std::string & fallback) const = 0;
  virtual std::vector<double> getDoubles(const std::string & key, const std::vector<double> & fallback) const = 0;
  virtual std::vector<std::string> getStrings(
    const std::string & key, const std::vector<std::string> & fallback) const = 0;
};

/// Over maps, for tests.
class MapParams : public Params
{
public:
  std::map<std::string, double> doubles;
  std::map<std::string, std::string> strings;
  std::map<std::string, std::vector<double>> vectors;

  bool has(const std::string & key) const override
  {
    return doubles.count(key) || strings.count(key) || vectors.count(key);
  }
  double getDouble(const std::string & key, double fallback) const override
  {
    auto it = doubles.find(key);
    return it == doubles.end() ? fallback : it->second;
  }
  bool getBool(const std::string & key, bool fallback) const override
  {
    auto it = doubles.find(key);
    return it == doubles.end() ? fallback : it->second != 0.0;
  }
  std::string getString(const std::string & key, const std::string & fallback) const override
  {
    auto it = strings.find(key);
    return it == strings.end() ? fallback : it->second;
  }
  std::vector<double> getDoubles(const std::string & key, const std::vector<double> & fallback) const override
  {
    auto it = vectors.find(key);
    return it == vectors.end() ? fallback : it->second;
  }
  std::vector<std::string> getStrings(const std::string &, const std::vector<std::string> & fallback) const override
  {
    return fallback;
  }
};

}  // namespace sac_perception
