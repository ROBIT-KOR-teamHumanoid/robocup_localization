#include "robocup_localization/ros/qnode.hpp"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

std::string QNode::resolveModelPath(const std::string &path) const
{
    if (path.empty())
        return path;

    if (path[0] == '/')
        return path;

    if (path.rfind("~/", 0) == 0)
    {
        const char *home = std::getenv("HOME");
        if (home && *home)
            return std::string(home) + path.substr(1);
        return path;
    }

    const std::string base =
        ament_index_cpp::get_package_share_directory("robocup_localization");
    return (std::filesystem::path(base) / path).string();
}

void QNode::getParams()
{
    constexpr int default_particle_count = 8000;

    node->declare_parameter("topic.sub_topics.imu", "Imu");
    node->declare_parameter("topic.sub_topics.vision", "vision");
    node->declare_parameter("topic.sub_topics.vision_feature", "vision_feature");
    node->declare_parameter("topic.sub_topics.ik", "ikcoordinate");
    node->declare_parameter("topic.sub_topics.game_controller", "gamecontroldata");
    node->declare_parameter("topic.sub_topics.udp", "udp");
    node->declare_parameter("topic.sub_topics.task_planner", "master2local");
    node->declare_parameter("topic.pub_topics.localization", "localization");
    node->declare_parameter("odometry.input", "ik");
    node->declare_parameter("particle_filter.min_fit", 0.3);
    node->declare_parameter("particle_filter.threads", 4);
    node->declare_parameter(
        "particle_filter.particle_nums", default_particle_count);
    node->declare_parameter("particle_filter.noise_enable", true);
    node->declare_parameter("rate.ros_spin_hz", 20.0);
    node->declare_parameter("rate.ui_data_hz", 20.0);
    node->declare_parameter("rate.ui_render_hz", 5.0);
    node->declare_parameter("inference.mlp_enable", false);
    node->declare_parameter("inference.reloc_enable", false);
    node->declare_parameter("inference.mlp_model", "");
    node->declare_parameter("inference.reloc_model", "");
    node->declare_parameter("inference.reloc_topk", 30);
    node->declare_parameter("kidnap.enable", false);
    node->declare_parameter("kidnap.disagree_dist_m", 2.0);
    node->declare_parameter("kidnap.cluster_radius_m", 0.5);
    node->declare_parameter("kidnap.min_frames", 5);
    node->declare_parameter("kidnap.cooldown_sec", 3.0);
    node->declare_parameter("kidnap.verify", true);
    node->declare_parameter("kidnap.verify_ratio", 1.5);
    node->declare_parameter("kidnap.verify_min_score", 0.3);
    node->declare_parameter("latency_compensation.enable", true);
    node->declare_parameter("latency_compensation.history_sec", 1.0);
    node->declare_parameter("latency_compensation.max_age_sec", 0.5);

    imu_topic = node->get_parameter("topic.sub_topics.imu").as_string();
    vision_topic = node->get_parameter("topic.sub_topics.vision").as_string();
    vision_feature_topic =
        node->get_parameter("topic.sub_topics.vision_feature").as_string();
    ik_topic = node->get_parameter("topic.sub_topics.ik").as_string();
    game_control_topic =
        node->get_parameter("topic.sub_topics.game_controller").as_string();
    udp_topic = node->get_parameter("topic.sub_topics.udp").as_string();
    master_topic = node->get_parameter("topic.sub_topics.task_planner").as_string();
    local_pub_topic = node->get_parameter("topic.pub_topics.localization").as_string();

    const std::string odometry_input =
        node->get_parameter("odometry.input").as_string();
    if (odometry_input == "metric")
    {
        robot0.odom_input = ROBOT::OdometryInput::Metric;
        RCLCPP_INFO(node->get_logger(),
                    "odometry.input: metric (ikcoordinate 를 cm 실제 이동량으로 쓴다, odom 계수 무시)");
    }
    else if (odometry_input == "ik")
    {
        robot0.odom_input = ROBOT::OdometryInput::Ik;
        RCLCPP_INFO(node->get_logger(),
                    "odometry.input: ik (PARAM 파일의 방향별 odom 계수를 쓴다)");
    }
    else
    {
        throw std::runtime_error(
            "odometry.input must be \"ik\" or \"metric\", got \"" + odometry_input + "\"");
    }

    robocup_localization::core::ParticleFilterConfig config;
    config.particle_count = static_cast<std::size_t>(
        node->get_parameter("particle_filter.particle_nums").as_int());
    config.min_fit = node->get_parameter("particle_filter.min_fit").as_double();
    config.particle_range = particle_range_;
    config.noise_enabled = node->get_parameter(
                                   "particle_filter.noise_enable")
                               .as_bool();
    config.threads = static_cast<int>(
        node->get_parameter("particle_filter.threads").as_int());
    localization_engine_.configureParticleFilter(config);
    RCLCPP_INFO(node->get_logger(), "[mcl] particles %zu, weighting threads %d (0 = all cores)",
                config.particle_count, config.threads);

    auto &loop_rate = localization_config_.loop_rate;
    loop_rate.ros_spin_hz =
        node->get_parameter("rate.ros_spin_hz").as_double();
    loop_rate.ui_data_hz =
        node->get_parameter("rate.ui_data_hz").as_double();
    loop_rate.ui_render_hz =
        node->get_parameter("rate.ui_render_hz").as_double();

    if (loop_rate.ros_spin_hz <= 0.0)
        loop_rate.ros_spin_hz = 20.0;
    if (loop_rate.ui_data_hz <= 0.0)
        loop_rate.ui_data_hz = 20.0;
    if (loop_rate.ui_render_hz <= 0.0)
        loop_rate.ui_render_hz = 5.0;

    inference_config_.mlp_enable =
        node->get_parameter("inference.mlp_enable").as_bool();
    inference_config_.reloc_enable =
        node->get_parameter("inference.reloc_enable").as_bool();
    inference_config_.mlp_model_path =
        resolveModelPath(node->get_parameter("inference.mlp_model").as_string());
    inference_config_.reloc_model_path =
        resolveModelPath(node->get_parameter("inference.reloc_model").as_string());
    inference_config_.reloc_topk =
        node->get_parameter("inference.reloc_topk").as_int();

    latency_compensation_ =
        node->get_parameter("latency_compensation.enable").as_bool();
    const double history_sec =
        node->get_parameter("latency_compensation.history_sec").as_double();
    const double max_age_sec =
        node->get_parameter("latency_compensation.max_age_sec").as_double();
    yaw_history_.setSpan(static_cast<std::int64_t>(history_sec * 1e9));
    max_feature_age_ns_ = static_cast<std::int64_t>(max_age_sec * 1e9);
    if (latency_compensation_)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "[latency] on: features use the yaw at capture time (history %.2fs, max age %.2fs)",
            history_sec, max_age_sec);
    }

    robocup_localization::app::KidnapRecoveryConfig kidnap_config;
    kidnap_config.enable =
        node->get_parameter("kidnap.enable").as_bool();
    kidnap_config.disagree_dist_m =
        node->get_parameter("kidnap.disagree_dist_m").as_double();
    kidnap_config.cluster_radius_m =
        node->get_parameter("kidnap.cluster_radius_m").as_double();
    kidnap_config.min_frames =
        static_cast<int>(node->get_parameter("kidnap.min_frames").as_int());
    kidnap_config.cooldown_sec =
        node->get_parameter("kidnap.cooldown_sec").as_double();
    kidnap_config.verify =
        node->get_parameter("kidnap.verify").as_bool();
    kidnap_config.verify_ratio =
        node->get_parameter("kidnap.verify_ratio").as_double();
    kidnap_config.verify_min_score =
        node->get_parameter("kidnap.verify_min_score").as_double();

    // Below MAX_JUMP the PF corrects itself; judging kidnaps there would overwrite a healthy PF.
    if (kidnap_config.disagree_dist_m < MclTune::MAX_JUMP_M)
    {
        RCLCPP_WARN(
            node->get_logger(),
            "[kidnap] disagree_dist_m %.2f < MAX_JUMP_M %.2f — %.2f 로 올립니다",
            kidnap_config.disagree_dist_m,
            MclTune::MAX_JUMP_M,
            MclTune::MAX_JUMP_M);
        kidnap_config.disagree_dist_m = MclTune::MAX_JUMP_M;
    }
    kidnap_.configure(kidnap_config);

    if (kidnap_config.enable)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "[kidnap] on: dist>=%.2fm, %d frames, cluster<=%.2fm, cooldown %.1fs",
            kidnap_config.disagree_dist_m, kidnap_config.min_frames,
            kidnap_config.cluster_radius_m, kidnap_config.cooldown_sec);
        if (kidnap_config.verify)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "[kidnap] verify: score>=%.2f and >=x%.2f current",
                kidnap_config.verify_min_score, kidnap_config.verify_ratio);
        }
    }

    // Must match robocup_task_planner state_pose.yaml (passed in by the launch file).
    auto &initial_pose = localization_config_.initial_pose;
    for (int i = 0; i < robocup_localization::app::InitialPoseConfig::ROBOT_COUNT; ++i)
    {
        const std::string prefix =
            "initial_pose.robot" + std::to_string(i + 1) + ".";

        node->declare_parameter(prefix + "x", 0.0);
        node->declare_parameter(prefix + "y", 0.0);
        node->declare_parameter(prefix + "yaw_deg", 0.0);

        initial_pose.left[i].x = node->get_parameter(prefix + "x").as_double();
        initial_pose.left[i].y = node->get_parameter(prefix + "y").as_double();
        initial_pose.left[i].yaw_deg =
            node->get_parameter(prefix + "yaw_deg").as_double();
    }

    const std::pair<bool, std::string> model_checks[] = {
        {inference_config_.mlp_enable, inference_config_.mlp_model_path},
        {inference_config_.reloc_enable, inference_config_.reloc_model_path}};
    const char *model_names[] = {"mlp", "reloc"};

    for (int i = 0; i < 2; ++i)
    {
        const auto &[enabled, path] = model_checks[i];
        if (enabled && !std::filesystem::exists(path))
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "[inference] %s 모델을 찾을 수 없습니다: %s",
                model_names[i], path.c_str());
        }
    }
}
