#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = MY_POINT_REG_SOURCE_DIR "/output_test_file";

// Clamps a (possibly short or out-of-range) 3-element parameter array to a
// valid 0-255 RGB triple. Missing channels come out 0 rather than throwing,
// since a malformed color parameter should paint a cloud black, not fail the
// whole merge.
std::array<uint8_t, 3> clampToRgb(const std::vector<int64_t> & channels)
{
  std::array<uint8_t, 3> rgb{0, 0, 0};
  for (std::size_t i = 0; i < 3 && i < channels.size(); ++i) {
    rgb[i] = static_cast<uint8_t>(std::clamp<int64_t>(channels[i], 0, 255));
  }
  return rgb;
}

// Reads a 4x4 transform written in the same plain-text, space-separated,
// row-major layout fine_registration_node writes (see
// POINTCLOUD_MERGE_PLAN.md's Open3D cross-check note). Returns false (with
// `error_message` set) on any missing/short/malformed file.
bool readTransform(
  const std::string & path, Eigen::Matrix4f & transform,
  std::string & error_message)
{
  std::ifstream transform_file(path);
  if (!transform_file.is_open()) {
    error_message = "Failed to open transform file: " + path;
    return false;
  }
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!(transform_file >> transform(row, col))) {
        error_message = "Transform file does not contain a 4x4 matrix: " + path;
        return false;
      }
    }
  }
  // operator>> parses "nan" and "inf" without complaint. Transforming the
  // source cloud by one writes a .pcd full of NaN points that still reports a
  // plausible point count, so reject it here.
  if (!transform.allFinite()) {
    error_message = "Transform file contains non-finite values: " + path;
    return false;
  }
  if (!transform.row(3).transpose().isApprox(
      Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), 1e-4f))
  {
    error_message =
      "Transform file is not an affine 4x4 transform (bottom row is not "
      "[0 0 0 1]): " + path;
    return false;
  }
  return true;
}
}  // namespace

// "transform + merge" stage of the point-cloud merge pipeline (see
// POINTCLOUD_MERGE_PLAN.md, Step 5). File in, file out: this node does
// nothing on startup, declares its parameters, and does all its work inside
// the `merge` Trigger callback so a human decides when it runs.
//
// Transforms the source cloud into the target cloud's frame using
// fine_registration_node's output_transform_path, then concatenates
// target + transformed source into a single merged cloud. Does not
// deduplicate overlapping points - that is Step 6.
class MergeNode : public rclcpp::Node
{
public:
  MergeNode()
  : Node("merge_node")
  {
    this->declare_parameter<std::string>("source_pcd_path", "");
    this->declare_parameter<std::string>("target_pcd_path", "");
    this->declare_parameter<std::string>("transform_path", "");
    this->declare_parameter<std::string>("output_merged_pcd_path", "");

    // Optional side output: the same merge, but as an XYZRGB cloud with each
    // input cloud painted a solid color so the two scans are visually
    // distinguishable in a viewer (RViz, pcl_viewer, CloudCompare, ...).
    // Left empty (the default), no colored copy is written - the plain
    // merged cloud below is what dedup_node reads, and RGB fields would only
    // confuse it. 0-255 per channel; out-of-range values are clamped.
    this->declare_parameter<std::string>("output_colored_pcd_path", "");
    this->declare_parameter<std::vector<int64_t>>(
      "source_color_rgb", {255, 60, 60});   // red
    this->declare_parameter<std::vector<int64_t>>(
      "target_color_rgb", {60, 160, 255});  // blue

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "merge",
      std::bind(
        &MergeNode::handleMerge, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handleMerge(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string source_pcd_path =
      this->get_parameter("source_pcd_path").as_string();
    const std::string target_pcd_path =
      this->get_parameter("target_pcd_path").as_string();
    const std::string transform_path =
      this->get_parameter("transform_path").as_string();
    const std::string output_merged_pcd_path =
      this->get_parameter("output_merged_pcd_path").as_string();
    const std::string output_colored_pcd_path =
      this->get_parameter("output_colored_pcd_path").as_string();

    if (source_pcd_path.empty() || target_pcd_path.empty() ||
      transform_path.empty())
    {
      response->success = false;
      response->message =
        "'source_pcd_path', 'target_pcd_path' and 'transform_path' "
        "parameters must all be set.";
      return;
    }

    std::string resolved_output_merged_pcd_path = output_merged_pcd_path;
    if (resolved_output_merged_pcd_path.empty()) {
      const std::filesystem::path source_path(source_pcd_path);
      const std::filesystem::path output_dir(kDefaultOutputDir);
      std::filesystem::create_directories(output_dir);
      resolved_output_merged_pcd_path =
        (output_dir /
        (source_path.stem().string() + "_merged.pcd")).string();
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    std::string transform_error;
    if (!readTransform(transform_path, transform, transform_error)) {
      response->success = false;
      response->message = transform_error;
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(
      new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(source_pcd_path, *source) == -1) {
      response->success = false;
      response->message =
        "Failed to load source PCD file: " + source_pcd_path;
      return;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(target_pcd_path, *target) == -1) {
      response->success = false;
      response->message =
        "Failed to load target PCD file: " + target_pcd_path;
      return;
    }

    if (source->empty() || target->empty()) {
      response->success = false;
      response->message = "One or more input PCD files is empty.";
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> source_aligned;
    pcl::transformPointCloud(*source, source_aligned, transform);

    pcl::PointCloud<pcl::PointXYZI> merged;
    merged = *target;
    merged += source_aligned;

    if (pcl::io::savePCDFileBinary(
        resolved_output_merged_pcd_path, merged) != 0)
    {
      response->success = false;
      response->message =
        "Failed to save output merged PCD file: " +
        resolved_output_merged_pcd_path;
      return;
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded target (" << target->size() << " pts) and source ("
        << source->size() << " pts) -> transformed source into target frame "
      "-> merged (" << merged.size() << " pts) -> "
        << resolved_output_merged_pcd_path;

    // Colored side output: target and the transformed source painted as two
    // solid colors in one XYZRGB cloud, purely so the overlap/alignment is
    // visible by eye. Kept separate from `merged` above (which stays
    // PointXYZI) because dedup_node reads that file next and has no use for
    // an RGB field.
    if (!output_colored_pcd_path.empty()) {
      const auto target_rgb = clampToRgb(
        this->get_parameter("target_color_rgb").as_integer_array());
      const auto source_rgb = clampToRgb(
        this->get_parameter("source_color_rgb").as_integer_array());

      pcl::PointCloud<pcl::PointXYZRGB> colored;
      colored.reserve(target->size() + source_aligned.size());
      for (const auto & pt : target->points) {
        pcl::PointXYZRGB c;
        c.x = pt.x;
        c.y = pt.y;
        c.z = pt.z;
        c.r = target_rgb[0];
        c.g = target_rgb[1];
        c.b = target_rgb[2];
        colored.push_back(c);
      }
      for (const auto & pt : source_aligned.points) {
        pcl::PointXYZRGB c;
        c.x = pt.x;
        c.y = pt.y;
        c.z = pt.z;
        c.r = source_rgb[0];
        c.g = source_rgb[1];
        c.b = source_rgb[2];
        colored.push_back(c);
      }
      colored.width = colored.size();
      colored.height = 1;
      colored.is_dense = merged.is_dense;

      if (pcl::io::savePCDFileBinary(output_colored_pcd_path, colored) != 0) {
        response->success = false;
        response->message =
          "Failed to save output colored PCD file: " +
          output_colored_pcd_path;
        return;
      }
      msg << "\nColored preview (target=rgb(" <<
        static_cast<int>(target_rgb[0]) << "," <<
        static_cast<int>(target_rgb[1]) << "," <<
        static_cast<int>(target_rgb[2]) << "), source=rgb(" <<
        static_cast<int>(source_rgb[0]) << "," <<
        static_cast<int>(source_rgb[1]) << "," <<
        static_cast<int>(source_rgb[2]) << ")): " << output_colored_pcd_path;
    }

    response->success = true;
    response->message = msg.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeNode>());
  rclcpp::shutdown();
  return 0;
}
