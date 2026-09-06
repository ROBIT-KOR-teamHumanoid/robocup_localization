#include "robocup_localization/ros/qnode.hpp"

#include <cstdlib>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"

// 패키지 share 기준 상대경로 설정
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

    node->declare_parameter("robot_feature", "kids");
    node->declare_parameter("topic.sub_topics.imu", "Imu");
    node->declare_parameter("topic.sub_topics.vision", "vision");
    node->declare_parameter("topic.sub_topics.vision_feature", "vision_feature");
    node->declare_parameter("topic.sub_topics.ik", "ikcoordinate");
    node->declare_parameter("topic.sub_topics.game_controller", "gamecontroldata");
    node->declare_parameter("topic.sub_topics.udp", "udp");
    node->declare_parameter("topic.sub_topics.task_planner", "master2local");
    node->declare_parameter("topic.pub_topics.localization", "localization");
    node->declare_parameter(
        "particle_filter.weight_update_threshold", 50.0);
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
    node->declare_parameter("inference.feature_min", 2);
    node->declare_parameter("inference.sign_right", -1.0);
    node->declare_parameter("inference.sign_forward", 1.0);
    node->declare_parameter("inference.yaw_offset_deg", 90.0);
    node->declare_parameter("inference.pan_yaw_sign", -1.0);
    node->declare_parameter("kidnap.enable", false);
    node->declare_parameter("kidnap.disagree_dist_m", 2.0);
    node->declare_parameter("kidnap.cluster_radius_m", 0.5);
    node->declare_parameter("kidnap.min_frames", 5);
    node->declare_parameter("kidnap.cooldown_sec", 3.0);

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
    robot_feature = node->get_parameter("robot_feature").as_string();

    robocup_localization::core::ParticleFilterConfig config;
    config.particle_count = static_cast<std::size_t>(
        node->get_parameter("particle_filter.particle_nums").as_int());
    config.weight_update_threshold = node->get_parameter(
                                             "particle_filter.weight_update_threshold")
                                         .as_double();
    config.particle_range = particle_range_;
    config.noise_enabled = node->get_parameter(
                                   "particle_filter.noise_enable")
                               .as_bool();
    localization_engine_.configureParticleFilter(config);

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
    inference_config_.feature_min =
        static_cast<int>(node->get_parameter("inference.feature_min").as_int());
    inference_config_.sign_right =
        node->get_parameter("inference.sign_right").as_double();
    inference_config_.sign_forward =
        node->get_parameter("inference.sign_forward").as_double();
    inference_config_.yaw_offset_deg =
        node->get_parameter("inference.yaw_offset_deg").as_double();
    inference_config_.pan_yaw_sign =
        node->get_parameter("inference.pan_yaw_sign").as_double();

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
    kidnap_.configure(kidnap_config);

    if (kidnap_config.enable)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "[kidnap] on: dist>=%.2fm, %d frames, cluster<=%.2fm, cooldown %.1fs",
            kidnap_config.disagree_dist_m, kidnap_config.min_frames,
            kidnap_config.cluster_radius_m, kidnap_config.cooldown_sec);
    }

    // INITIAL 배치. robocup_task_planner 의 state_pose.yaml 과 같은 값을
    // 써야 한다 — 심판이 로봇을 놓는 자리는 하나뿐이고, 추정기와 판단기가
    // 서로 다른 자리를 알고 있으면 READY 로 넘어갈 때 어긋난다. 런치가 그
    // yaml 을 읽어 여기로 넘긴다. yaml 은 왼쪽 진영만 적고, 오른쪽은
    // InitialPoseConfig::forSide 가 180도 돌려 만든다.
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

    // 없는 파일은 알림
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
