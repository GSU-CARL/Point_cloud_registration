// Tutorial-derived from code_reference/lec11_icp.cpp:
// align a source cloud onto a target cloud with pcl::IterativeClosestPoint
// and report the resulting 4x4 transform. Here source/target come from two
// point_lio-saved .pcd files on disk instead of the tutorial's synthetic
// KITTI-derived pair, and alignment runs on a service trigger instead of
// main()'s CloudViewer loop (which this node drops entirely).
//
// Beyond the tutorial: once ICP has converged the aligned source sits in the
// target's frame, so the two clouds can be concatenated into a single map.
// That is opt-in via 'output_merged_pcd_path'.
#include <cstddef>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using PointT = pcl::PointXYZI;

class IcpNode : public rclcpp::Node {
public:
  IcpNode() : Node("icp_node") {
    declare_parameter<std::string>("input_source_pcd_path", "");
    declare_parameter<std::string>("input_target_pcd_path", "");
    declare_parameter<std::string>("output_pcd_path", "");
    // Empty path disables the merge; merged_leaf_size <= 0 keeps every merged
    // point (leaving duplicates wherever the clouds overlap).
    declare_parameter<std::string>("output_merged_pcd_path", "");
    declare_parameter<double>("merged_leaf_size", 0.0);
    declare_parameter<double>("max_correspondence_distance", 1.0);
    declare_parameter<double>("transformation_epsilon", 0.003);
    declare_parameter<int>("max_iterations", 1000);

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/align",
        std::bind(&IcpNode::handleTrigger, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "icp_node ready. Call '%s' to align source onto target.",
                trigger_srv_->get_service_name());
  }

private:
  void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    const std::string input_source_pcd_path = get_parameter("input_source_pcd_path").as_string();
    const std::string input_target_pcd_path = get_parameter("input_target_pcd_path").as_string();

    if (input_source_pcd_path.empty() || input_target_pcd_path.empty()) {
      response->success = false;
      response->message = "Parameters 'input_source_pcd_path' and 'input_target_pcd_path' must both be set.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (!std::filesystem::exists(input_source_pcd_path)) {
      response->success = false;
      response->message = "input_source_pcd_path does not exist: " + input_source_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (!std::filesystem::exists(input_target_pcd_path)) {
      response->success = false;
      response->message = "input_target_pcd_path does not exist: " + input_target_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(input_source_pcd_path, *cloud_src) < 0) {
      response->success = false;
      response->message = "Failed to load source PCD file: " + input_source_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (pcl::io::loadPCDFile<PointT>(input_target_pcd_path, *cloud_tgt) < 0) {
      response->success = false;
      response->message = "Failed to load target PCD file: " + input_target_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(get_parameter("max_correspondence_distance").as_double());
    icp.setTransformationEpsilon(get_parameter("transformation_epsilon").as_double());
    icp.setMaximumIterations(get_parameter("max_iterations").as_int());

    pcl::PointCloud<PointT>::Ptr cloud_aligned(new pcl::PointCloud<PointT>);
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.align(*cloud_aligned);

    const Eigen::Matrix4f src2tgt = icp.getFinalTransformation();
    const double score = icp.getFitnessScore();
    const bool is_converged = icp.hasConverged();

    if (!is_converged) {
      response->success = false;
      response->message = "ICP did not converge (fitness score=" + std::to_string(score) + ").";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string output_pcd_path = get_parameter("output_pcd_path").as_string();
    if (output_pcd_path.empty()) {
      std::filesystem::path in_path(input_source_pcd_path);
      output_pcd_path = (in_path.parent_path() / (in_path.stem().string() + "_aligned.pcd")).string();
    }
    if (pcl::io::savePCDFileBinary(output_pcd_path, *cloud_aligned) < 0) {
      response->success = false;
      response->message = "Failed to save aligned PCD to: " + output_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string merge_summary;
    if (!mergeIntoMap(cloud_aligned, cloud_tgt, merge_summary, response->message)) {
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    std::ostringstream transform_str;
    transform_str << src2tgt;

    response->success = true;
    response->message = "Converged (fitness score=" + std::to_string(score) + ") -> " + output_pcd_path +
                         merge_summary + "\nsrc2tgt transform:\n" + transform_str.str();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  // Concatenates the aligned source onto the target and saves the result as a
  // single map. No-op (returns true, leaves summary empty) when
  // 'output_merged_pcd_path' is unset. On failure returns false with the reason
  // in err.
  bool mergeIntoMap(const pcl::PointCloud<PointT>::Ptr &cloud_aligned, const pcl::PointCloud<PointT>::Ptr &cloud_tgt,
                     std::string &summary, std::string &err) {
    const std::string output_merged_pcd_path = get_parameter("output_merged_pcd_path").as_string();
    if (output_merged_pcd_path.empty()) {
      return true;
    }

    // Safe because ICP has already put cloud_aligned in the target's frame.
    pcl::PointCloud<PointT>::Ptr cloud_merged(new pcl::PointCloud<PointT>(*cloud_aligned));
    *cloud_merged += *cloud_tgt;
    const std::size_t concatenated_size = cloud_merged->size();

    // Concatenation duplicates points wherever the two clouds overlap; a voxel
    // pass at the leaf size the inputs were downsampled with collapses them.
    const double merged_leaf_size = get_parameter("merged_leaf_size").as_double();
    if (merged_leaf_size > 0.0) {
      pcl::PointCloud<PointT>::Ptr cloud_merged_filtered(new pcl::PointCloud<PointT>);
      pcl::VoxelGrid<PointT> voxel_filter;
      voxel_filter.setInputCloud(cloud_merged);
      voxel_filter.setLeafSize(static_cast<float>(merged_leaf_size), static_cast<float>(merged_leaf_size),
                                static_cast<float>(merged_leaf_size));
      voxel_filter.filter(*cloud_merged_filtered);
      cloud_merged = cloud_merged_filtered;
    }

    if (pcl::io::savePCDFileBinary(output_merged_pcd_path, *cloud_merged) < 0) {
      err = "Failed to save merged PCD to: " + output_merged_pcd_path;
      return false;
    }

    summary = "\nMerged aligned source + target = " + std::to_string(concatenated_size) + " points";
    if (merged_leaf_size > 0.0) {
      summary += ", voxelized to " + std::to_string(cloud_merged->size()) +
                 " (leaf_size=" + std::to_string(merged_leaf_size) + ")";
    }
    summary += " -> " + output_merged_pcd_path;
    return true;
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpNode>());
  rclcpp::shutdown();
  return 0;
}
