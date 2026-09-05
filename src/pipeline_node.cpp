#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = MY_POINT_REG_SOURCE_DIR "/output_test_file";

// One entry per stage node this pipeline drives: the parameter client used to
// reconfigure it before a run, and the Trigger client used to run it. Both
// live on the pipeline node's client callback group so their responses are
// serviced by a different executor thread than the `run_pipeline` callback
// that blocks on them.
struct Stage
{
  std::string node_name;
  rclcpp::AsyncParametersClient::SharedPtr parameters;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger;
};
}  // namespace

// End-to-end driver for the point-cloud merge pipeline (see
// POINTCLOUD_MERGE_PLAN.md and MERGE_WORKFLOW.md). Unlike the stage nodes it
// touches no point data itself: it owns the sequence and the intermediate file
// paths, sets each stage node's parameters over the parameter service, then
// calls that node's Trigger service and waits for the result.
//
// The sequence, source aligned onto target throughout:
//   1. voxel_node             source  -> <source>_voxelized.pcd
//   2. voxel_node             target  -> <target>_voxelized.pcd
//   3. feature_node           voxelized source -> _normals.pcd + _fpfh.pcd
//   4. feature_node           voxelized target -> _normals.pcd + _fpfh.pcd
//   5. coarse_registration_node  normals + FPFH -> _coarse_transform.txt
//   6. fine_registration_node    voxelized clouds + coarse transform
//                                -> _fine_transform.txt
//   7. merge_node                voxelized clouds + fine transform -> _merged.pcd
//   8. dedup_node                 merged cloud -> _deduplicated.pcd
//
// Steps 3-5 exist only to give GICP an initial guess, so how many of them run
// depends on how that guess is produced:
//
//   coarse_method=yaw_sweep (default) - 5 steps + dedup. The sweep works off
//     the voxelized clouds directly, so both feature stages drop out.
//   coarse_method=sac_ia               - 7 steps + dedup, as listed above.
//   skip_coarse=true                   - 4 steps + dedup, GICP starts from
//     identity.
//
// See the `skip_coarse` and `coarse_method` declarations for how to choose.
//
// Step 8 (deduplicating the merged cloud, Plan Step 6) re-runs voxel_node's
// filter over the merge output to collapse points both scans contributed in
// the overlapping region. It runs by default; set `skip_dedup` true to leave
// the merged cloud as-is.
//
// All six stage nodes must already be running, one instance each in this
// node's namespace: `ros2 launch my_point_reg pipeline.launch.py`.
class PipelineNode : public rclcpp::Node
{
public:
  PipelineNode()
  : Node("pipeline_node")
  {
    this->declare_parameter<std::string>("source_pcd_path", "");
    this->declare_parameter<std::string>("target_pcd_path", "");
    this->declare_parameter<std::string>("output_dir", kDefaultOutputDir);
    this->declare_parameter<std::string>("output_merged_pcd_path", "");

    // Skips coarse registration and seeds GICP with identity instead. Leave
    // false unless the pair is already known to be near-aligned: identity is
    // only a safe guess when it is already almost the answer, and GICP cannot
    // walk back a large yaw error no matter how the correspondence gate is
    // scheduled. PCD/ground.pcd and PCD/air.pcd are 87 degrees and 47 m apart,
    // which is what a skipped coarse stage looks like from the far end.
    this->declare_parameter<bool>("skip_coarse", false);

    // Which coarse method runs - see coarse_registration_node's own comment.
    // 'yaw_sweep' consumes the voxelized clouds directly and needs no
    // descriptors, so it skips the two feature stages with it; 'sac_ia' needs
    // both feature stages to run first.
    this->declare_parameter<std::string>("coarse_method", "yaw_sweep");

    // Steps 1-2: voxel_node.
    this->declare_parameter<double>("leaf_size", 0.1);
    this->declare_parameter<bool>("remove_outliers", false);
    this->declare_parameter<int>("sor_mean_k", 50);
    this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);

    // Steps 3-4: feature_node.
    this->declare_parameter<double>("normal_radius", 0.5);
    this->declare_parameter<double>("fpfh_radius", 1.0);
    this->declare_parameter<int>("feature_num_threads", 4);
    // Leave true: PCL's default origin viewpoint makes ground-plane normals a
    // coin flip, which makes FPFH descriptors incomparable between clouds.
    this->declare_parameter<bool>("orient_normals_up", true);

    // Step 5: coarse_registration_node, method 'yaw_sweep'.
    this->declare_parameter<double>("coarse_sweep_min_z", 0.6);
    this->declare_parameter<double>("coarse_sweep_max_z", 4.0);
    this->declare_parameter<double>("coarse_sweep_yaw_min_deg", 0.0);
    this->declare_parameter<double>("coarse_sweep_yaw_max_deg", 360.0);
    this->declare_parameter<double>("coarse_sweep_yaw_step_deg", 1.0);
    this->declare_parameter<double>("coarse_sweep_vote_cell_size", 1.0);
    this->declare_parameter<double>("coarse_sweep_cell_size", 0.25);
    this->declare_parameter<int>("coarse_sweep_refine_candidates", 8);
    this->declare_parameter<double>("coarse_min_overlap_score", 0.0);

    // Step 5: coarse_registration_node, method 'sac_ia'.
    this->declare_parameter<double>("coarse_min_sample_distance", 0.05);
    this->declare_parameter<double>("coarse_max_correspondence_distance", 1.0);
    this->declare_parameter<int>("coarse_nr_iterations", 500);
    this->declare_parameter<int>("coarse_number_of_samples", 3);
    this->declare_parameter<int>("coarse_correspondence_randomness", 10);
    this->declare_parameter<double>("coarse_max_fitness_score", 0.0);

    // Step 6: fine_registration_node (GICP). One pass per entry, tightening;
    // see fine_registration_node's own comment for why a single loose gate
    // lets a ground-plane-dominated cloud slide.
    this->declare_parameter<std::vector<double>>(
      "fine_max_correspondence_distances", {0.5, 0.3, 0.2});
    this->declare_parameter<double>("fine_transformation_epsilon", 1e-8);
    this->declare_parameter<double>("fine_euclidean_fitness_epsilon", 1e-6);
    this->declare_parameter<double>("fine_rotation_epsilon", 2e-3);
    this->declare_parameter<int>("fine_max_iterations", 50);
    this->declare_parameter<int>("fine_correspondence_randomness", 20);
    // Both gates default to 0 (disabled) because a useful threshold depends
    // on the scene scale and leaf_size. Do one baseline run, read the two
    // fitness scores out of the /run_pipeline response, then set these just
    // above them so a bad registration stops the run instead of quietly
    // seeding the next stage with a wrong transform.
    this->declare_parameter<double>("fine_max_fitness_score", 0.0);

    // Step 7: merge_node's colored preview. Purely visual - target and the
    // aligned source painted as two solid colors in a separate XYZRGB file
    // so the merge/overlap is visible by eye, alongside the plain merged
    // cloud dedup_node reads. See merge_node's own comment for why the two
    // outputs are kept separate.
    this->declare_parameter<bool>("colorize_merge", true);
    this->declare_parameter<std::vector<int64_t>>(
      "merge_source_color_rgb", {255, 60, 60});   // red
    this->declare_parameter<std::vector<int64_t>>(
      "merge_target_color_rgb", {60, 160, 255});  // blue

    // Step 8: dedup_node. Runs by default - see the class comment above for
    // why leaving it on is the right default now that the alignment problem
    // is being tracked separately from deduplication.
    this->declare_parameter<bool>("skip_dedup", false);
    this->declare_parameter<double>("dedup_leaf_size", 0.1);
    this->declare_parameter<int>("dedup_min_points_per_voxel", 1);

    // Orchestration. SAC-IA and GICP on multi-million-point clouds are slow,
    // so the per-stage budget is generous by default.
    this->declare_parameter<double>("stage_timeout_sec", 1800.0);
    this->declare_parameter<double>("discovery_timeout_sec", 10.0);

    client_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    voxel_ = makeStage("voxel_node", "voxelize");
    feature_ = makeStage("feature_node", "estimate_features");
    coarse_ = makeStage("coarse_registration_node", "register_coarse");
    fine_ = makeStage("fine_registration_node", "register_fine");
    merge_ = makeStage("merge_node", "merge");
    dedup_ = makeStage("dedup_node", "deduplicate");

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "run_pipeline",
      std::bind(
        &PipelineNode::handleRunPipeline, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), service_callback_group_);
  }

private:
  Stage makeStage(
    const std::string & node_name, const std::string & service_name)
  {
    Stage stage;
    stage.node_name = node_name;
    stage.parameters = std::make_shared<rclcpp::AsyncParametersClient>(
      this, node_name, rclcpp::ParametersQoS(), client_callback_group_);
    stage.trigger = this->create_client<std_srvs::srv::Trigger>(
      service_name, rclcpp::ServicesQoS(), client_callback_group_);
    return stage;
  }

  // Pushes `parameters` onto the stage node, calls its Trigger service, and
  // appends the stage's own response message to `log`. Returns false with
  // `error_message` set on any discovery timeout, rejected parameter, call
  // timeout, or a stage reporting success == false.
  bool runStage(
    const Stage & stage, const std::string & label,
    const std::vector<rclcpp::Parameter> & parameters,
    std::ostringstream & log, std::string & error_message)
  {
    const auto discovery_timeout = std::chrono::duration<double>(
      this->get_parameter("discovery_timeout_sec").as_double());
    const auto stage_timeout = std::chrono::duration<double>(
      this->get_parameter("stage_timeout_sec").as_double());

    if (!stage.parameters->wait_for_service(discovery_timeout)) {
      error_message = label + ": parameter services of '" + stage.node_name +
        "' are not available - is it running? (ros2 launch my_point_reg "
        "pipeline.launch.py)";
      return false;
    }
    if (!stage.trigger->wait_for_service(discovery_timeout)) {
      error_message = label + ": service '" +
        std::string(stage.trigger->get_service_name()) +
        "' is not available - is '" + stage.node_name + "' running?";
      return false;
    }

    // Reconfiguring a stage is a round trip that takes milliseconds, so it
    // gets the discovery budget rather than the (30 minute) stage budget - a
    // node that died between the wait_for_service above and here then surfaces
    // in seconds instead of stalling the whole run.
    auto set_future = stage.parameters->set_parameters(parameters);
    if (set_future.wait_for(discovery_timeout) != std::future_status::ready) {
      error_message = label + ": timed out setting parameters on '" +
        stage.node_name + "' within discovery_timeout_sec";
      return false;
    }
    const auto results = set_future.get();
    const std::size_t checked = std::min(results.size(), parameters.size());
    for (std::size_t i = 0; i < checked; ++i) {
      if (!results[i].successful) {
        error_message = label + ": '" + stage.node_name + "' rejected "
          "parameter '" + parameters[i].get_name() + "': " +
          results[i].reason;
        return false;
      }
    }

    const auto started = std::chrono::steady_clock::now();
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto trigger_future = stage.trigger->async_send_request(request);
    if (trigger_future.wait_for(stage_timeout) != std::future_status::ready) {
      stage.trigger->remove_pending_request(trigger_future);
      error_message = label + ": '" + stage.node_name +
        "' did not respond within stage_timeout_sec";
      return false;
    }
    const auto response = trigger_future.get();
    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();

    if (!response->success) {
      error_message = label + ": " + response->message;
      return false;
    }

    log << label << " (" << elapsed_sec << "s): " << response->message << "\n";
    RCLCPP_INFO(
      this->get_logger(), "%s done in %.1fs", label.c_str(), elapsed_sec);
    return true;
  }

  void handleRunPipeline(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string source_pcd_path =
      this->get_parameter("source_pcd_path").as_string();
    const std::string target_pcd_path =
      this->get_parameter("target_pcd_path").as_string();
    const std::string output_dir_param =
      this->get_parameter("output_dir").as_string();
    const std::string output_merged_pcd_path =
      this->get_parameter("output_merged_pcd_path").as_string();

    if (source_pcd_path.empty()) {
      response->success = false;
      response->message = "'source_pcd_path' parameter must be set.";
      return;
    }
    if (target_pcd_path.empty()) {
      response->success = false;
      response->message = "'target_pcd_path' parameter must be set.";
      return;
    }
    if (!std::filesystem::exists(source_pcd_path)) {
      response->success = false;
      response->message = "Source PCD file does not exist: " + source_pcd_path;
      return;
    }
    if (!std::filesystem::exists(target_pcd_path)) {
      response->success = false;
      response->message = "Target PCD file does not exist: " + target_pcd_path;
      return;
    }

    const double leaf_size = this->get_parameter("leaf_size").as_double();
    const bool remove_outliers =
      this->get_parameter("remove_outliers").as_bool();
    const int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
    const double sor_stddev_mul_thresh =
      this->get_parameter("sor_stddev_mul_thresh").as_double();
    const double normal_radius =
      this->get_parameter("normal_radius").as_double();
    const double fpfh_radius = this->get_parameter("fpfh_radius").as_double();
    const int feature_num_threads =
      this->get_parameter("feature_num_threads").as_int();
    const bool skip_coarse = this->get_parameter("skip_coarse").as_bool();
    const std::string coarse_method =
      this->get_parameter("coarse_method").as_string();
    const bool skip_dedup = this->get_parameter("skip_dedup").as_bool();
    const double dedup_leaf_size =
      this->get_parameter("dedup_leaf_size").as_double();
    const int dedup_min_points_per_voxel =
      this->get_parameter("dedup_min_points_per_voxel").as_int();

    if (!skip_coarse && coarse_method != "yaw_sweep" &&
      coarse_method != "sac_ia")
    {
      response->success = false;
      response->message =
        "'coarse_method' must be 'yaw_sweep' or 'sac_ia' (got '" +
        coarse_method + "').";
      return;
    }

    // The feature stages exist only to feed SAC-IA, so they run only for that
    // method - yaw_sweep works off the voxelized clouds directly. Stage labels
    // are numbered against the count that will actually run so the log reads
    // correctly in all four cases (three coarse variants, times dedup on/off).
    const bool run_features = !skip_coarse && coarse_method == "sac_ia";
    const int total_steps = (skip_coarse ? 4 : (run_features ? 7 : 5)) +
      (skip_dedup ? 0 : 1);
    int step = 0;
    const auto label = [&step, total_steps](const std::string & name) {
        return std::to_string(++step) + "/" + std::to_string(total_steps) +
               " " + name;
      };

    // PCL degrades silently rather than erroring on these: a leaf_size of 0
    // makes VoxelGrid pass the cloud through at full resolution, and FPFH
    // over the millions of points that follow looks like a hang rather than a
    // bad parameter. Catch them before any stage runs.
    std::vector<std::pair<const char *, double>> positive_params{
      {"leaf_size", leaf_size},
      {"stage_timeout_sec",
        this->get_parameter("stage_timeout_sec").as_double()},
      {"discovery_timeout_sec",
        this->get_parameter("discovery_timeout_sec").as_double()}};
    // Only meaningful when the feature stages run.
    if (run_features) {
      positive_params.emplace_back("normal_radius", normal_radius);
      positive_params.emplace_back("fpfh_radius", fpfh_radius);
    }
    // Only meaningful when the dedup stage runs.
    if (!skip_dedup) {
      positive_params.emplace_back("dedup_leaf_size", dedup_leaf_size);
    }
    if (!skip_dedup && dedup_min_points_per_voxel < 1) {
      response->success = false;
      response->message =
        "'dedup_min_points_per_voxel' must be >= 1 (got " +
        std::to_string(dedup_min_points_per_voxel) + ").";
      return;
    }
    for (const auto & [name, value] : positive_params) {
      if (value <= 0.0) {
        std::ostringstream bad;
        bad << std::fixed << std::setprecision(6) << "'" << name <<
          "' must be greater than 0 (got " << value << ").";
        response->success = false;
        response->message = bad.str();
        return;
      }
    }

    const std::filesystem::path output_dir(
      output_dir_param.empty() ? kDefaultOutputDir : output_dir_param);
    std::filesystem::create_directories(output_dir);

    const std::string source_stem =
      std::filesystem::path(source_pcd_path).stem().string();
    const std::string target_stem =
      std::filesystem::path(target_pcd_path).stem().string();

    // Every per-cloud intermediate below is named from the stem alone, so two
    // inputs sharing one (/runA/scan.pcd + /runB/scan.pcd) would have step 2
    // overwrite step 1's output and the run would go on to register the
    // target against itself - reporting success, a near-identity transform
    // and a fitness score of roughly zero.
    if (source_stem == target_stem) {
      response->success = false;
      response->message =
        "Source and target file stems are both '" + source_stem +
        "' - the voxelized/normals/FPFH intermediates would collide. Rename "
        "one input so the two stems differ.";
      return;
    }

    const std::string pair_stem = source_stem + "_to_" + target_stem;

    // Every intermediate the stages hand to each other is named here rather
    // than left to the stage nodes' own empty-parameter defaults, so one run
    // is self-consistent even when output_dir is not the default.
    const std::string source_voxelized =
      (output_dir / (source_stem + "_voxelized.pcd")).string();
    const std::string target_voxelized =
      (output_dir / (target_stem + "_voxelized.pcd")).string();
    const std::string source_normals =
      (output_dir / (source_stem + "_normals.pcd")).string();
    const std::string source_fpfh =
      (output_dir / (source_stem + "_fpfh.pcd")).string();
    const std::string target_normals =
      (output_dir / (target_stem + "_normals.pcd")).string();
    const std::string target_fpfh =
      (output_dir / (target_stem + "_fpfh.pcd")).string();
    const std::string coarse_aligned =
      (output_dir / (pair_stem + "_coarse_aligned.pcd")).string();
    const std::string coarse_transform =
      (output_dir / (pair_stem + "_coarse_transform.txt")).string();
    const std::string fine_aligned =
      (output_dir / (pair_stem + "_fine_aligned.pcd")).string();
    const std::string fine_transform =
      (output_dir / (pair_stem + "_fine_transform.txt")).string();
    const std::string merged = output_merged_pcd_path.empty() ?
      (output_dir / (pair_stem + "_merged.pcd")).string() :
      output_merged_pcd_path;
    const std::string deduplicated =
      (output_dir / (pair_stem + "_deduplicated.pcd")).string();
    const std::string merged_colored =
      (output_dir / (pair_stem + "_merged_colored.pcd")).string();

    // output_dir is created above, but a custom output_merged_pcd_path can
    // point anywhere. PCL's savePCDFileBinary throws pcl::IOException on a
    // missing directory instead of returning non-zero, which would take
    // merge_node's process down and leave this node waiting out the full
    // stage_timeout before blaming the timeout rather than the path.
    const auto merged_parent = std::filesystem::path(merged).parent_path();
    if (!merged_parent.empty()) {
      std::filesystem::create_directories(merged_parent);
    }

    std::ostringstream log;
    log << std::fixed << std::setprecision(6);
    std::string error_message;

    const std::vector<rclcpp::Parameter> voxel_common{
      rclcpp::Parameter("leaf_size", leaf_size),
      rclcpp::Parameter("remove_outliers", remove_outliers),
      rclcpp::Parameter("sor_mean_k", sor_mean_k),
      rclcpp::Parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh)};

    // Step 1: voxelize the source cloud.
    std::vector<rclcpp::Parameter> stage_params = voxel_common;
    stage_params.emplace_back("input_pcd_path", source_pcd_path);
    stage_params.emplace_back("output_pcd_path", source_voxelized);
    if (!runStage(voxel_, label("voxelize source"), stage_params, log,
      error_message))
    {
      failPipeline(response, log, error_message);
      return;
    }

    // Step 2: voxelize the target cloud.
    stage_params = voxel_common;
    stage_params.emplace_back("input_pcd_path", target_pcd_path);
    stage_params.emplace_back("output_pcd_path", target_voxelized);
    if (!runStage(voxel_, label("voxelize target"), stage_params, log,
      error_message))
    {
      failPipeline(response, log, error_message);
      return;
    }

    // The coarse stage exists only to produce GICP's initial guess, and the
    // two feature stages exist only to feed the sac_ia flavour of it. With
    // skip_coarse everything here is skipped and GICP starts from identity -
    // see the `skip_coarse` declaration for why that is not the default.
    if (skip_coarse) {
      log << "Skipped feature estimation and coarse registration "
        "(skip_coarse is true): GICP starts from identity.\n";
    } else if (run_features) {
      const std::vector<rclcpp::Parameter> feature_common{
        rclcpp::Parameter("normal_radius", normal_radius),
        rclcpp::Parameter("fpfh_radius", fpfh_radius),
        rclcpp::Parameter("num_threads", feature_num_threads),
        rclcpp::Parameter(
          "orient_normals_up",
          this->get_parameter("orient_normals_up").as_bool())};

      // Step 3: normals + FPFH on the voxelized source.
      stage_params = feature_common;
      stage_params.emplace_back("input_pcd_path", source_voxelized);
      stage_params.emplace_back("output_normals_pcd_path", source_normals);
      stage_params.emplace_back("output_fpfh_pcd_path", source_fpfh);
      if (!runStage(feature_, label("features source"), stage_params, log,
        error_message))
      {
        failPipeline(response, log, error_message);
        return;
      }

      // Step 4: normals + FPFH on the voxelized target.
      stage_params = feature_common;
      stage_params.emplace_back("input_pcd_path", target_voxelized);
      stage_params.emplace_back("output_normals_pcd_path", target_normals);
      stage_params.emplace_back("output_fpfh_pcd_path", target_fpfh);
      if (!runStage(feature_, label("features target"), stage_params, log,
        error_message))
      {
        failPipeline(response, log, error_message);
        return;
      }

      // Step 5: coarse registration (SAC-IA) -> initial guess for GICP.
      stage_params = {
        rclcpp::Parameter("method", coarse_method),
        rclcpp::Parameter("source_normals_pcd_path", source_normals),
        rclcpp::Parameter("source_fpfh_pcd_path", source_fpfh),
        rclcpp::Parameter("target_normals_pcd_path", target_normals),
        rclcpp::Parameter("target_fpfh_pcd_path", target_fpfh),
        rclcpp::Parameter("output_aligned_pcd_path", coarse_aligned),
        rclcpp::Parameter("output_transform_path", coarse_transform),
        rclcpp::Parameter(
          "min_sample_distance",
          this->get_parameter("coarse_min_sample_distance").as_double()),
        rclcpp::Parameter(
          "max_correspondence_distance",
          this->get_parameter(
            "coarse_max_correspondence_distance").as_double()),
        rclcpp::Parameter(
          "nr_iterations",
          this->get_parameter("coarse_nr_iterations").as_int()),
        rclcpp::Parameter(
          "number_of_samples",
          this->get_parameter("coarse_number_of_samples").as_int()),
        rclcpp::Parameter(
          "correspondence_randomness",
          this->get_parameter("coarse_correspondence_randomness").as_int()),
        rclcpp::Parameter(
          "max_fitness_score",
          this->get_parameter("coarse_max_fitness_score").as_double())};
      if (!runStage(coarse_, label("coarse registration"), stage_params, log,
        error_message))
      {
        failPipeline(response, log, error_message);
        return;
      }
    } else {
      // Step 3: coarse registration (yaw sweep) -> initial guess for GICP.
      // No feature stages: the sweep scores 2D occupancy overlap over the
      // voxelized clouds themselves, so it needs neither normals nor FPFH.
      stage_params = {
        rclcpp::Parameter("method", coarse_method),
        rclcpp::Parameter("source_pcd_path", source_voxelized),
        rclcpp::Parameter("target_pcd_path", target_voxelized),
        rclcpp::Parameter("output_aligned_pcd_path", coarse_aligned),
        rclcpp::Parameter("output_transform_path", coarse_transform),
        rclcpp::Parameter(
          "sweep_min_z",
          this->get_parameter("coarse_sweep_min_z").as_double()),
        rclcpp::Parameter(
          "sweep_max_z",
          this->get_parameter("coarse_sweep_max_z").as_double()),
        rclcpp::Parameter(
          "sweep_yaw_min_deg",
          this->get_parameter("coarse_sweep_yaw_min_deg").as_double()),
        rclcpp::Parameter(
          "sweep_yaw_max_deg",
          this->get_parameter("coarse_sweep_yaw_max_deg").as_double()),
        rclcpp::Parameter(
          "sweep_yaw_step_deg",
          this->get_parameter("coarse_sweep_yaw_step_deg").as_double()),
        rclcpp::Parameter(
          "sweep_vote_cell_size",
          this->get_parameter("coarse_sweep_vote_cell_size").as_double()),
        rclcpp::Parameter(
          "sweep_cell_size",
          this->get_parameter("coarse_sweep_cell_size").as_double()),
        rclcpp::Parameter(
          "sweep_refine_candidates",
          this->get_parameter("coarse_sweep_refine_candidates").as_int()),
        rclcpp::Parameter(
          "min_overlap_score",
          this->get_parameter("coarse_min_overlap_score").as_double())};
      if (!runStage(coarse_, label("coarse registration"), stage_params, log,
        error_message))
      {
        failPipeline(response, log, error_message);
        return;
      }
    }

    // Step 6: fine registration (GICP) on the voxelized clouds, seeded with
    // the coarse transform - or with identity when skip_coarse dropped it.
    stage_params = {
      rclcpp::Parameter("source_pcd_path", source_voxelized),
      rclcpp::Parameter("target_pcd_path", target_voxelized),
      rclcpp::Parameter(
        "initial_guess_transform_path",
        skip_coarse ? std::string() : coarse_transform),
      rclcpp::Parameter("output_aligned_pcd_path", fine_aligned),
      rclcpp::Parameter("output_transform_path", fine_transform),
      rclcpp::Parameter(
        "max_correspondence_distances",
        this->get_parameter(
          "fine_max_correspondence_distances").as_double_array()),
      rclcpp::Parameter(
        "transformation_epsilon",
        this->get_parameter("fine_transformation_epsilon").as_double()),
      rclcpp::Parameter(
        "euclidean_fitness_epsilon",
        this->get_parameter("fine_euclidean_fitness_epsilon").as_double()),
      rclcpp::Parameter(
        "rotation_epsilon",
        this->get_parameter("fine_rotation_epsilon").as_double()),
      rclcpp::Parameter(
        "max_iterations",
        this->get_parameter("fine_max_iterations").as_int()),
      rclcpp::Parameter(
        "correspondence_randomness",
        this->get_parameter("fine_correspondence_randomness").as_int()),
      rclcpp::Parameter(
        "max_fitness_score",
        this->get_parameter("fine_max_fitness_score").as_double())};
    if (!runStage(fine_, label("fine registration"), stage_params, log,
      error_message))
    {
      failPipeline(response, log, error_message);
      return;
    }

    // Step 7: merge the voxelized clouds using the fine transform.
    const bool colorize_merge =
      this->get_parameter("colorize_merge").as_bool();
    stage_params = {
      rclcpp::Parameter("source_pcd_path", source_voxelized),
      rclcpp::Parameter("target_pcd_path", target_voxelized),
      rclcpp::Parameter("transform_path", fine_transform),
      rclcpp::Parameter("output_merged_pcd_path", merged),
      rclcpp::Parameter(
        "output_colored_pcd_path",
        colorize_merge ? merged_colored : std::string()),
      rclcpp::Parameter(
        "source_color_rgb",
        this->get_parameter("merge_source_color_rgb").as_integer_array()),
      rclcpp::Parameter(
        "target_color_rgb",
        this->get_parameter("merge_target_color_rgb").as_integer_array())};
    if (!runStage(merge_, label("merge"), stage_params, log, error_message)) {
      failPipeline(response, log, error_message);
      return;
    }

    // Step 8: deduplicate the merged cloud - re-runs voxel_node's filter over
    // the merge output to collapse points both scans contributed in the
    // overlapping region (Plan Step 6).
    if (skip_dedup) {
      log << "Skipped deduplication (skip_dedup is true): merged cloud is "
        "written as-is.\n";
    } else {
      stage_params = {
        rclcpp::Parameter("input_pcd_path", merged),
        rclcpp::Parameter("output_pcd_path", deduplicated),
        rclcpp::Parameter("leaf_size", dedup_leaf_size),
        rclcpp::Parameter(
          "min_points_per_voxel", dedup_min_points_per_voxel)};
      if (!runStage(dedup_, label("deduplicate"), stage_params, log,
        error_message))
      {
        failPipeline(response, log, error_message);
        return;
      }
    }

    log << "Merged cloud: " << merged;
    if (!skip_dedup) {
      log << "\nDeduplicated cloud: " << deduplicated;
    }
    if (colorize_merge) {
      log << "\nColored preview: " << merged_colored;
    }
    response->success = true;
    response->message = log.str();
  }

  // Reports a stage failure with the log of the stages that did run, so a
  // partial pipeline is still diagnosable from the Trigger response alone.
  void failPipeline(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response,
    const std::ostringstream & log, const std::string & error_message)
  {
    response->success = false;
    response->message = log.str() + "FAILED at " + error_message;
    RCLCPP_ERROR(
      this->get_logger(), "Pipeline failed at %s", error_message.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  Stage voxel_;
  Stage feature_;
  Stage coarse_;
  Stage fine_;
  Stage merge_;
  Stage dedup_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // At least two threads: the `run_pipeline` callback blocks on futures that
  // only complete when another thread services the stage clients' responses.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<PipelineNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
