#include "sac_perception/robot_geometry.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>

namespace sac_perception
{

namespace
{
Eigen::Isometry3f toIsometry(const urdf::Pose & pose)
{
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translation() = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
  t.linear() = Eigen::Quaternionf(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z)
    .normalized().toRotationMatrix();
  return t;
}

/// The link's pose in the URDF's root, joints at zero
Eigen::Isometry3f linkInRoot(const urdf::LinkConstSharedPtr & link)
{
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  for (auto l = link; l && l->parent_joint; l = l->getParent()) {
    pose = toIsometry(l->parent_joint->parent_to_joint_origin_transform) * pose;
  }
  return pose;
}

std::string resolve(const std::string & uri)
{
  const std::string package = "package://";
  const std::string file = "file://";
  if (uri.rfind(package, 0) == 0) {
    const std::string rest = uri.substr(package.size());
    const std::size_t slash = rest.find('/');
    if (slash == std::string::npos) {
      return {};
    }
    try {
      return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) + rest.substr(slash);
    } catch (const std::exception &) {
      return {};
    }
  }
  if (uri.rfind(file, 0) == 0) {
    return uri.substr(file.size());
  }
  return uri;
}

/// The vertices' bounds of an STL (binary or ASCII) or OBJ file; false if unreadable
bool meshBounds(const std::string & path, Eigen::Vector3f & lo, Eigen::Vector3f & hi)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
  hi = -lo;
  bool any = false;
  auto add = [&](float x, float y, float z) {
    lo = lo.cwiseMin(Eigen::Vector3f(x, y, z));
    hi = hi.cwiseMax(Eigen::Vector3f(x, y, z));
    any = true;
  };
  std::string lower = path;
  for (char & c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  const auto ends = [&](const std::string & suffix) {
    return lower.size() >= suffix.size() && lower.compare(lower.size() - suffix.size(), suffix.size(), suffix) == 0;
  };
  if (ends(".stl")) {
    uint32_t triangles = 0;
    if (data.size() >= 84) {
      std::memcpy(&triangles, data.data() + 80, 4);
    }
    if (data.size() >= 84 && data.size() == 84 + 50ull * triangles) {  // binary
      for (uint32_t t = 0; t < triangles; ++t) {
        const char * p = data.data() + 84 + 50ull * t + 12;  // after the normal
        for (int v = 0; v < 3; ++v) {
          float xyz[3];
          std::memcpy(xyz, p + 12 * v, 12);
          add(xyz[0], xyz[1], xyz[2]);
        }
      }
    } else {  // ASCII: "vertex x y z"
      std::istringstream text(data);
      std::string word;
      while (text >> word) {
        if (word == "vertex") {
          float x, y, z;
          if (text >> x >> y >> z) {
            add(x, y, z);
          }
        }
      }
    }
  } else if (ends(".obj")) {
    std::istringstream text(data);
    std::string line;
    while (std::getline(text, line)) {
      if (line.size() > 2 && line[0] == 'v' && line[1] == ' ') {
        std::istringstream v(line.substr(2));
        float x, y, z;
        if (v >> x >> y >> z) {
          add(x, y, z);
        }
      }
    }
  }
  return any;
}
}  // namespace

std::optional<RobotGeometry> vehicleFromUrdf(const std::string & xml, const std::string & base_frame)
{
  urdf::Model model;
  if (!model.initString(xml)) {
    return std::nullopt;
  }
  const urdf::LinkConstSharedPtr base = model.getLink(base_frame);
  if (!base) {
    return std::nullopt;
  }
  const Eigen::Isometry3f root_to_base = linkInRoot(base).inverse();
  RobotGeometry result;
  Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
  Eigen::Vector3f hi = -lo;
  auto addBox = [&](const Eigen::Isometry3f & pose, const Eigen::Vector3f & a, const Eigen::Vector3f & b) {
    for (int c = 0; c < 8; ++c) {
      const Eigen::Vector3f corner((c & 1) ? b.x() : a.x(), (c & 2) ? b.y() : a.y(), (c & 4) ? b.z() : a.z());
      const Eigen::Vector3f p = pose * corner;
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
  };

  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  for (const auto & link : links) {
    const Eigen::Isometry3f link_pose = root_to_base * linkInRoot(link);
    std::vector<std::pair<urdf::Pose, urdf::GeometrySharedPtr>> shapes;
    for (const auto & v : link->visual_array) {
      if (v && v->geometry) {
        shapes.emplace_back(v->origin, v->geometry);
      }
    }
    for (const auto & c : link->collision_array) {
      if (c && c->geometry) {
        shapes.emplace_back(c->origin, c->geometry);
      }
    }
    for (const auto & [origin, geometry] : shapes) {
      const Eigen::Isometry3f pose = link_pose * toIsometry(origin);
      switch (geometry->type) {
        case urdf::Geometry::BOX: {
          const auto & d = std::static_pointer_cast<urdf::Box>(geometry)->dim;
          const Eigen::Vector3f half(d.x / 2, d.y / 2, d.z / 2);
          addBox(pose, -half, half);
          break;
        }
        case urdf::Geometry::CYLINDER: {
          const auto c = std::static_pointer_cast<urdf::Cylinder>(geometry);
          const Eigen::Vector3f half(c->radius, c->radius, c->length / 2);
          addBox(pose, -half, half);
          break;
        }
        case urdf::Geometry::SPHERE: {
          const float r = std::static_pointer_cast<urdf::Sphere>(geometry)->radius;
          addBox(pose, Eigen::Vector3f::Constant(-r), Eigen::Vector3f::Constant(r));
          break;
        }
        case urdf::Geometry::MESH: {
          const auto m = std::static_pointer_cast<urdf::Mesh>(geometry);
          Eigen::Vector3f a, b;
          if (meshBounds(resolve(m->filename), a, b)) {
            const Eigen::Vector3f scale(m->scale.x, m->scale.y, m->scale.z);
            const Eigen::Vector3f sa = a.cwiseProduct(scale);
            const Eigen::Vector3f sb = b.cwiseProduct(scale);
            addBox(pose, sa.cwiseMin(sb), sa.cwiseMax(sb));
          } else {
            result.warnings.push_back(link->name + ": " + m->filename);
            addBox(pose, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
          }
          break;
        }
      }
    }
  }
  if (!std::isfinite(lo.x())) {
    return std::nullopt;  // no shapes at all
  }
  result.box.min = lo;
  result.box.max = hi;
  result.box.known = true;
  return result;
}

}  // namespace sac_perception
