#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include "geometry_msgs/msg/point.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{
struct Transform3
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};

Transform3 compose(const Transform3 & a, const Transform3 & b)
{
  Transform3 out;
  out.R = a.R * b.R;
  out.t = a.R * b.t + a.t;
  return out;
}

Eigen::Vector3d apply(const Transform3 & tf, const Eigen::Vector3d & p)
{
  return tf.R * p + tf.t;
}

std::vector<double> parseDoubles(const std::string & s)
{
  std::istringstream iss(s);
  std::vector<double> vals;
  double v = 0.0;
  while (iss >> v) {
    vals.push_back(v);
  }
  return vals;
}

Transform3 parsePoseElement(const tinyxml2::XMLElement * pose_elem)
{
  Transform3 tf;
  if (!pose_elem || !pose_elem->GetText()) {
    return tf;
  }
  const auto vals = parseDoubles(pose_elem->GetText());
  if (vals.size() < 6) {
    return tf;
  }
  const double roll = vals[3];
  const double pitch = vals[4];
  const double yaw = vals[5];
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  tf.R = (rz * ry * rx).toRotationMatrix();
  tf.t = Eigen::Vector3d(vals[0], vals[1], vals[2]);
  return tf;
}
}  // namespace

class WorldToOctomapNode : public rclcpp::Node
{
public:
  WorldToOctomapNode()
  : Node("world_to_octomap")
  {
    declare_parameter<std::string>("world_file", "");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("xy_window_size_m", 24.0);
    declare_parameter<double>("ground_surface_max_thickness_m", 0.6);
    declare_parameter<bool>("enable_stair_step_surface_mode", true);
    declare_parameter<double>("stair_step_max_height_m", 0.5);
    declare_parameter<double>("stair_step_max_depth_m", 0.8);
    declare_parameter<double>("stair_step_min_width_m", 1.0);
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/octomap_occupied_markers");
    declare_parameter<std::string>("world_file_cmd_topic", "/world_file_cmd");

    const auto world_file = get_parameter("world_file").as_string();
    half_xy_extent_m_ = 0.5 * get_parameter("xy_window_size_m").as_double();
    ground_surface_max_thickness_m_ = get_parameter("ground_surface_max_thickness_m").as_double();
    enable_stair_step_surface_mode_ = get_parameter("enable_stair_step_surface_mode").as_bool();
    stair_step_max_height_m_ = get_parameter("stair_step_max_height_m").as_double();
    stair_step_max_depth_m_ = get_parameter("stair_step_max_depth_m").as_double();
    stair_step_min_width_m_ = get_parameter("stair_step_min_width_m").as_double();

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("marker_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    world_file_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("world_file_cmd_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&WorldToOctomapNode::onWorldFileCmd, this, std::placeholders::_1));

    if (!world_file.empty()) {
      loadWorld(world_file);
    } else {
      RCLCPP_WARN(get_logger(), "No initial world_file set. Waiting for /world_file_cmd.");
    }
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&WorldToOctomapNode::publishAll, this));
  }

private:
  void onWorldFileCmd(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string file = msg->data;
    if (file.empty()) {
      return;
    }
    if (file == loaded_world_file_) {
      RCLCPP_INFO(get_logger(), "Reload requested for same world file: %s", file.c_str());
    }
    loadWorld(file);
  }

  void loadWorld(const std::string & world_file)
  {
    const double resolution = get_parameter("resolution").as_double();
    half_xy_extent_m_ = 0.5 * get_parameter("xy_window_size_m").as_double();
    try {
      generateFromWorld(world_file, resolution);
      loaded_world_file_ = world_file;
      publishAll();
      RCLCPP_INFO(get_logger(), "Loaded world file: %s", world_file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Load world failed: %s", e.what());
    }
  }

  void generateFromWorld(const std::string & world_file, double resolution)
  {
    tree_ = std::make_shared<octomap::OcTree>(resolution);
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(world_file.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to load world/sdf file: %s", world_file.c_str());
      throw std::runtime_error("failed to load world file");
    }

    const tinyxml2::XMLElement * sdf = doc.FirstChildElement("sdf");
    if (!sdf) {
      RCLCPP_ERROR(get_logger(), "No <sdf> root in file: %s", world_file.c_str());
      throw std::runtime_error("invalid sdf");
    }
    const tinyxml2::XMLElement * world = sdf->FirstChildElement("world");
    if (!world) {
      RCLCPP_ERROR(get_logger(), "No <world> in sdf file: %s", world_file.c_str());
      throw std::runtime_error("invalid world");
    }

    Transform3 world_tf;
    int shape_count = 0;
    for (const tinyxml2::XMLElement * model = world->FirstChildElement("model");
      model; model = model->NextSiblingElement("model"))
    {
      const Transform3 model_tf = compose(world_tf, parsePoseElement(model->FirstChildElement("pose")));
      parseModel(model, model_tf, shape_count);
    }

    tree_->updateInnerOccupancy();
    RCLCPP_INFO(get_logger(), "World voxelization done. shapes=%d occupied_voxels=%zu", shape_count, tree_->size());
  }

  void parseModel(const tinyxml2::XMLElement * model, const Transform3 & model_tf, int & shape_count)
  {
    for (const tinyxml2::XMLElement * link = model->FirstChildElement("link");
      link; link = link->NextSiblingElement("link"))
    {
      const Transform3 link_tf = compose(model_tf, parsePoseElement(link->FirstChildElement("pose")));
      for (const tinyxml2::XMLElement * collision = link->FirstChildElement("collision");
        collision; collision = collision->NextSiblingElement("collision"))
      {
        const Transform3 col_tf = compose(link_tf, parsePoseElement(collision->FirstChildElement("pose")));
        const tinyxml2::XMLElement * geom = collision->FirstChildElement("geometry");
        if (!geom) {
          continue;
        }
        if (const auto * box = geom->FirstChildElement("box")) {
          fillBox(col_tf, box);
          ++shape_count;
        } else if (const auto * cyl = geom->FirstChildElement("cylinder")) {
          fillCylinder(col_tf, cyl);
          ++shape_count;
        } else if (const auto * sph = geom->FirstChildElement("sphere")) {
          fillSphere(col_tf, sph);
          ++shape_count;
        } else if (const auto * plane = geom->FirstChildElement("plane")) {
          fillPlane(col_tf, plane);
          ++shape_count;
        }
      }
    }
  }

  void markPoint(const Eigen::Vector3d & p)
  {
    if (half_xy_extent_m_ > 0.0 &&
      (std::abs(p.x()) > half_xy_extent_m_ || std::abs(p.y()) > half_xy_extent_m_))
    {
      return;
    }
    octomap::OcTreeKey key;
    const octomap::point3d q(
      static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    if (!tree_->coordToKeyChecked(q, key)) {
      return;
    }
    const octomap::point3d center = tree_->keyToCoord(key);
    tree_->updateNode(center, true);
  }

  void fillBox(const Transform3 & tf, const tinyxml2::XMLElement * box)
  {
    const auto * size_elem = box->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 3) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double sz = vals[2];
    const double r = tree_->getResolution();
    const double min_xy = std::min(sx, sy);
    const double max_xy = std::max(sx, sy);

    // Thin, near-horizontal boxes are treated as ground-like surfaces:
    // fill only the top surface densely so it is continuous and single-layer.
    const Eigen::Vector3d local_z_in_world = tf.R * Eigen::Vector3d::UnitZ();
    const bool near_horizontal = std::abs(local_z_in_world.z()) > 0.9;
    const bool thin_ground_like = near_horizontal && (sz <= ground_surface_max_thickness_m_);
    const bool stair_step_like =
      enable_stair_step_surface_mode_ && near_horizontal &&
      (sz <= stair_step_max_height_m_) &&
      (min_xy <= stair_step_max_depth_m_) &&
      (max_xy >= stair_step_min_width_m_);
    if (thin_ground_like || stair_step_like) {
      const double step = std::max(r * 0.5, 1e-3);
      const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
      const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
      for (int ix = 0; ix <= nx; ++ix) {
        const double x = -sx * 0.5 + (sx * static_cast<double>(ix) / static_cast<double>(nx));
        for (int iy = 0; iy <= ny; ++iy) {
          const double y = -sy * 0.5 + (sy * static_cast<double>(iy) / static_cast<double>(ny));
          markPoint(apply(tf, Eigen::Vector3d(x, y, sz * 0.5)));
        }
      }
      return;
    }

    // Use index-based sampling to avoid floating-step accumulation gaps.
    // For thin boxes (typical walls), oversample to improve continuity.
    const double min_dim = std::min({sx, sy, sz});
    const double step = (min_dim <= 4.0 * r) ? std::max(r * 0.5, 1e-3) : r;
    const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
    const int nz = std::max(1, static_cast<int>(std::ceil(sz / step)));

    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx * 0.5 + (sx * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy * 0.5 + (sy * static_cast<double>(iy) / static_cast<double>(ny));
        for (int iz = 0; iz <= nz; ++iz) {
          const double z = -sz * 0.5 + (sz * static_cast<double>(iz) / static_cast<double>(nz));
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillCylinder(const Transform3 & tf, const tinyxml2::XMLElement * cyl)
  {
    const auto * r_elem = cyl->FirstChildElement("radius");
    const auto * l_elem = cyl->FirstChildElement("length");
    if (!r_elem || !l_elem || !r_elem->GetText() || !l_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double length = std::stod(l_elem->GetText());
    const double res = tree_->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        if (x * x + y * y > radius * radius) {
          continue;
        }
        for (double z = -length * 0.5; z <= length * 0.5; z += res) {
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillSphere(const Transform3 & tf, const tinyxml2::XMLElement * sph)
  {
    const auto * r_elem = sph->FirstChildElement("radius");
    if (!r_elem || !r_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double res = tree_->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        for (double z = -radius; z <= radius; z += res) {
          if (x * x + y * y + z * z > radius * radius) {
            continue;
          }
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillPlane(const Transform3 & tf, const tinyxml2::XMLElement * plane)
  {
    const auto * size_elem = plane->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 2) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double r = tree_->getResolution();
    const double step = std::max(r * 0.5, 1e-3);
    const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx * 0.5 + (sx * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy * 0.5 + (sy * static_cast<double>(iy) / static_cast<double>(ny));
        markPoint(apply(tf, Eigen::Vector3d(x, y, 0.0)));
      }
    }
  }

  void publishAll()
  {
    if (!tree_) {
      return;
    }
    const auto stamp = now();
    const std::string frame_id = get_parameter("frame_id").as_string();

    octomap_msgs::msg::Octomap map_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to serialize generated OctoMap.");
      return;
    }
    map_msg.header.stamp = stamp;
    map_msg.header.frame_id = frame_id;
    octomap_pub_->publish(map_msg);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = tree_->getResolution();
    marker.scale.y = tree_->getResolution();
    marker.scale.z = tree_->getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;

    marker.points.reserve(tree_->size());
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      marker.points.push_back(p);
    }
    marker_pub_->publish(marker);
  }

  std::shared_ptr<octomap::OcTree> tree_;
  std::string loaded_world_file_;
  double half_xy_extent_m_{12.0};
  double ground_surface_max_thickness_m_{0.6};
  bool enable_stair_step_surface_mode_{true};
  double stair_step_max_height_m_{0.5};
  double stair_step_max_depth_m_{0.8};
  double stair_step_min_width_m_{1.0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr world_file_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
