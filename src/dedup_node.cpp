#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = "output_test_file";
}  // namespace

// "deduplicate" stage of the point-cloud merge pipeline (see
// POINTCLOUD_MERGE_PLAN.md, Step 6). File in, file out: this node does
// nothing on startup, declares its parameters, and does all its work inside
// the `deduplicate` Trigger callback so a human decides when it runs.
//
// Re-runs pcl::VoxelGrid (Step 1's filter) over the merged cloud to collapse
// points that both scans contributed in the overlapping region - the same
// filter, applied a second time, downstream of merge_node instead of
// upstream of registration.
class DedupNode : public rclcpp::Node
{
public:
  DedupNode()
  : Node("dedup_node")
  {
    this->declare_parameter<std::string>("input_pcd_path", "");
    this->declare_parameter<std::string>("output_pcd_path", "");
    this->declare_parameter<double>("leaf_size", 0.1);
    this->declare_parameter<int>("min_points_per_voxel", 1);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "deduplicate",
      std::bind(
        &DedupNode::handleDeduplicate, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handleDeduplicate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string input_pcd_path =
      this->get_parameter("input_pcd_path").as_string();
    const std::string output_pcd_path =
      this->get_parameter("output_pcd_path").as_string();
    const double leaf_size = this->get_parameter("leaf_size").as_double();
    const int min_points_per_voxel =
      this->get_parameter("min_points_per_voxel").as_int();

    if (input_pcd_path.empty()) {
      response->success = false;
      response->message = "'input_pcd_path' parameter must be set.";
      return;
    }

    // PCL degrades silently rather than erroring on either of these: a
    // leaf_size of 0 makes VoxelGrid pass the cloud through at full
    // resolution instead of deduplicating anything, and a negative
    // min_points_per_voxel is accepted as-is and treated like 0. Catch both
    // up front so a bad parameter reads as a rejected call, not a suspicious
    // point count.
    if (leaf_size <= 0.0) {
      std::ostringstream bad;
      bad << std::fixed << std::setprecision(6) <<
        "'leaf_size' must be greater than 0 (got " << leaf_size << ").";
      response->success = false;
      response->message = bad.str();
      return;
    }
    if (min_points_per_voxel < 1) {
      response->success = false;
      response->message =
        "'min_points_per_voxel' must be >= 1 (got " +
        std::to_string(min_points_per_voxel) + ").";
      return;
    }

    std::string resolved_output_pcd_path = output_pcd_path;
    if (resolved_output_pcd_path.empty()) {
      const std::filesystem::path input_path(input_pcd_path);
      const std::filesystem::path output_dir(kDefaultOutputDir);
      std::filesystem::create_directories(output_dir);
      resolved_output_pcd_path =
        (output_dir /
        (input_path.stem().string() + "_deduplicated.pcd")).string();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd_path, *cloud) == -1) {
      response->success = false;
      response->message = "Failed to load input PCD file: " + input_pcd_path;
      return;
    }

    if (cloud->empty()) {
      response->success = false;
      response->message = "Input PCD file is empty: " + input_pcd_path;
      return;
    }
    const std::size_t loaded_count = cloud->size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr deduplicated(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setMinimumPointsNumberPerVoxel(
      static_cast<std::size_t>(min_points_per_voxel));
    voxel_filter.filter(*deduplicated);
    const std::size_t deduplicated_count = deduplicated->size();

    if (pcl::io::savePCDFileBinary(
        resolved_output_pcd_path, *deduplicated) != 0)
    {
      response->success = false;
      response->message =
        "Failed to save output PCD file: " + resolved_output_pcd_path;
      return;
    }

    const std::size_t removed_count =
      loaded_count > deduplicated_count ? loaded_count - deduplicated_count :
      0;

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded " << loaded_count << " points -> deduplicated to "
        << deduplicated_count << " (removed " << removed_count
        << ", leaf_size=" << leaf_size << ", min_points_per_voxel="
        << min_points_per_voxel << ") -> " << resolved_output_pcd_path;

    response->success = true;
    response->message = msg.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DedupNode>());
  rclcpp::shutdown();
  return 0;
}
