// Params over maps, for tests.

#pragma once

#include <map>
#include <string>
#include <vector>

#include "sac_localization/core/params.hpp"

namespace sac_localization
{

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
  int getInt(const std::string & key, int fallback) const override
  {
    auto it = doubles.find(key);
    return it == doubles.end() ? fallback : static_cast<int>(it->second);
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

}  // namespace sac_localization
