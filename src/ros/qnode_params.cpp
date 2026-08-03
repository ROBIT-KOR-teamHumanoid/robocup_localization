#include "robocup_localization/ros/qnode.hpp"

void QNode::getParams()
{
    constexpr int default_particle_count = 8000;

    node->declare_parameter("robot_feature", "kids");
    node->declare_parameter("topic.imu_sub_topic", "Imu");
    node->declare_parameter("topic.vision_sub_topic", "vision");
    node->declare_parameter("topic.vision_feature_sub_topic", "vision_feature");
    node->declare_parameter("topic.ik_sub_topic", "ikcoordinate");
    node->declare_parameter("topic.game_control_sub_topic", "gamecontroldata");
    node->declare_parameter("topic.udp_sub_topic", "udp");
    node->declare_parameter("topic.master_sub_topic", "master2local");
    node->declare_parameter("topic.local_pub_topic", "localization");
    node->declare_parameter(
        "particle_filter.weight_update_threshold", 50.0);
    node->declare_parameter(
        "particle_filter.particle_nums", default_particle_count);
    node->declare_parameter("particle_filter.noise_enable", true);
    node->declare_parameter(
        "particle_filter.foot_align_odom_scale", 0.3);
    node->declare_parameter("setup_mode.Soccer_mode", true);
    node->declare_parameter("rate.ros_spin_hz", 20.0);
    node->declare_parameter("rate.ui_data_hz", 20.0);
    node->declare_parameter("rate.ui_render_hz", 5.0);

    imu_topic = node->get_parameter("topic.imu_sub_topic").as_string();
    vision_topic = node->get_parameter("topic.vision_sub_topic").as_string();
    vision_feature_topic =
        node->get_parameter("topic.vision_feature_sub_topic").as_string();
    ik_topic = node->get_parameter("topic.ik_sub_topic").as_string();
    game_control_topic =
        node->get_parameter("topic.game_control_sub_topic").as_string();
    udp_topic = node->get_parameter("topic.udp_sub_topic").as_string();
    master_topic = node->get_parameter("topic.master_sub_topic").as_string();
    local_pub_topic = node->get_parameter("topic.local_pub_topic").as_string();
    robot_feature = node->get_parameter("robot_feature").as_string();

    robocup_localization::core::ParticleFilterConfig config;
    config.particle_count = static_cast<std::size_t>(
        node->get_parameter("particle_filter.particle_nums").as_int());
    config.weight_update_threshold = node->get_parameter(
        "particle_filter.weight_update_threshold").as_double();
    config.particle_range = particle_range_;
    config.noise_enabled = node->get_parameter(
        "particle_filter.noise_enable").as_bool();
    localization_engine_.configureParticleFilter(config);

    foot_align_odom_scale_ = node->get_parameter(
        "particle_filter.foot_align_odom_scale").as_double();
    soccer_mode_ = node->get_parameter("setup_mode.Soccer_mode").as_bool();

    localization_config_.ros_spin_hz =
        node->get_parameter("rate.ros_spin_hz").as_double();
    localization_config_.ui_data_hz =
        node->get_parameter("rate.ui_data_hz").as_double();
    localization_config_.ui_render_hz =
        node->get_parameter("rate.ui_render_hz").as_double();

    if (localization_config_.ros_spin_hz <= 0.0)
        localization_config_.ros_spin_hz = 20.0;
    if (localization_config_.ui_data_hz <= 0.0)
        localization_config_.ui_data_hz = 20.0;
    if (localization_config_.ui_render_hz <= 0.0)
        localization_config_.ui_render_hz = 5.0;
}
