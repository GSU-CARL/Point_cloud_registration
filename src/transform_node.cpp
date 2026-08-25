// Tutorial-derived from code_reference/lec03_transformation.cpp:
// build a 4x4 Eigen transform from x,y,z + orientation and apply it with
// pcl::transformPointCloud. Here the x,y,z/orientation come from the drone's
// MAVROS pose instead of a hardcoded matrix, and the cloud comes from a
// point_lio-saved .pcd file on disk instead of an in-memory cloud.
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using PointT = pcl::PointXYZI;

class TransformNode : public rclcpp::Node {
public:
  TransformNode() : Node("transform_node") {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");
    declare_parameter<std::string>("input_pcd_path", "/home/fishman/ros2_ws/src/point_lio_ros2/PCD/scans.pcd");
    declare_parameter<std::string>("output_pcd_path", "");
    // Offline fallback for when nothing is publishing pose_topic: supply the
    // transform by hand instead. SI units, so metres and radians; the defaults
    // are identity, which makes the stage a pass-through copy. Opt-in rather
    // than an automatic fallback, so a merely-slow MAVROS can never silently
    // produce a cloud transformed by the wrong pose.
    declare_parameter<bool>("use_static_pose", false);
    declare_parameter<std::vector<double>>("static_translation_xyz", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("static_rotation_rpy", std::vector<double>{0.0, 0.0, 0.0});

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_pose_ = msg; });

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/transform",
        std::bind(&TransformNode::handleTrigger, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "transform_node ready. Listening for pose on '%s'; call '%s' to transform.",
                pose_topic_.c_str(), trigger_srv_->get_service_name());
  }

private:
  void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    const std::string input_pcd_path = get_parameter("input_pcd_path").as_string();

    if (input_pcd_path.empty()) {
      response->success = false;
      response->message = "Parameter 'input_pcd_path' is not set.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (!std::filesystem::exists(input_pcd_path)) {
      response->success = false;
      response->message = "input_pcd_path does not exist: " + input_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    // Resolve the transform before loading the cloud, so a missing pose fails
    // fast instead of after reading a multi-hundred-MB file.
    Eigen::Matrix4f trans;
    std::string pose_source;
    if (get_parameter("use_static_pose").as_bool()) {
      const std::vector<double> xyz = get_parameter("static_translation_xyz").as_double_array();
      const std::vector<double> rpy = get_parameter("static_rotation_rpy").as_double_array();
      if (xyz.size() != 3 || rpy.size() != 3) {
        response->success = false;
        response->message = "'static_translation_xyz' and 'static_rotation_rpy' must each have exactly 3 elements.";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
      trans = staticPoseToMatrix(xyz, rpy);
      pose_source = "static pose";
    } else if (!latest_pose_) {
      response->success = false;
      response->message = "No pose received yet on '" + pose_topic_ +
                          "'. Set 'use_static_pose' to true to supply the transform by hand instead.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    } else {
      trans = poseToMatrix(*latest_pose_);
      pose_source = "pose from '" + pose_topic_ + "'";
    }

    pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(input_pcd_path, *cloud_src) < 0) {
      response->success = false;
      response->message = "Failed to load PCD file: " + input_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud_src, *cloud_transformed, trans);

    std::string output_pcd_path = get_parameter("output_pcd_path").as_string();
    if (output_pcd_path.empty()) {
      std::filesystem::path in_path(input_pcd_path);
      output_pcd_path = (in_path.parent_path() / (in_path.stem().string() + "_transformed.pcd")).string();
    }

    if (pcl::io::savePCDFileBinary(output_pcd_path, *cloud_transformed) < 0) {
      response->success = false;
      response->message = "Failed to save transformed PCD to: " + output_pcd_path;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Transformed " + std::to_string(cloud_src->size()) + " points using " + pose_source + " -> " +
                         output_pcd_path;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  static Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::PoseStamped &pose) {
    const Eigen::Quaternionf q(static_cast<float>(pose.pose.orientation.w), static_cast<float>(pose.pose.orientation.x),
                                static_cast<float>(pose.pose.orientation.y), static_cast<float>(pose.pose.orientation.z));

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    trans(0, 3) = static_cast<float>(pose.pose.position.x);
    trans(1, 3) = static_cast<float>(pose.pose.position.y);
    trans(2, 3) = static_cast<float>(pose.pose.position.z);
    return trans;
  }

  static Eigen::Matrix4f staticPoseToMatrix(const std::vector<double> &xyz, const std::vector<double> &rpy) {
    // ROS convention for RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    const Eigen::Quaternionf q = Eigen::AngleAxisf(static_cast<float>(rpy[2]), Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(static_cast<float>(rpy[1]), Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(static_cast<float>(rpy[0]), Eigen::Vector3f::UnitX());

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    trans(0, 3) = static_cast<float>(xyz[0]);
    trans(1, 3) = static_cast<float>(xyz[1]);
    trans(2, 3) = static_cast<float>(xyz[2]);
    return trans;
  }

  std::string pose_topic_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformNode>());
  rclcpp::shutdown();
  return 0;
}
