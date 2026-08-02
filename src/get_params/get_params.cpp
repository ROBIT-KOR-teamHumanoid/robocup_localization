#include "../../include/robocup_localization/qnode.hpp"

void QNode::getParams()
{

    node->declare_parameter("robot_feature", "kids");
    node->declare_parameter("topic.imu_sub_topic", "Imu");
    node->declare_parameter("topic.vision_sub_topic", "vision");
    node->declare_parameter("topic.vision_feature_sub_topic", "vision_feature");
    node->declare_parameter("topic.ik_sub_topic", "ikcoordinate");
    node->declare_parameter("topic.game_control_sub_topic", "gamecontroldata");
    node->declare_parameter("topic.udp_sub_topic", "udp");
    node->declare_parameter("topic.master_sub_topic", "master2local");
    node->declare_parameter("topic.local_pub_topic", "localization");
    node->declare_parameter("particle_filter.weight_update_threshold", 50.0);
    node->declare_parameter("particle_filter.particle_nums", particle_num);
    node->declare_parameter("particle_filter.noise_enable", true);
    node->declare_parameter("particle_filter.foot_align_odom_scale", 0.3);
    node->declare_parameter("setup_mode.Soccer_mode", true);
    imu_topic = node->get_parameter("topic.imu_sub_topic").as_string();
    vision_topic = node->get_parameter("topic.vision_sub_topic").as_string();
    vision_feature_topic = node->get_parameter("topic.vision_feature_sub_topic").as_string();
    ik_topic = node->get_parameter("topic.ik_sub_topic").as_string();
    game_control_topic = node->get_parameter("topic.game_control_sub_topic").as_string();
    udp_topic = node->get_parameter("topic.udp_sub_topic").as_string();
    master_topic = node->get_parameter("topic.master_sub_topic").as_string();
    local_pub_topic = node->get_parameter("topic.local_pub_topic").as_string();

    // adult / kids different config
    robot_feature = node->get_parameter("robot_feature").as_string();

    weight_update_threshold = node->get_parameter("particle_filter.weight_update_threshold").as_double();
    particle_num = node->get_parameter("particle_filter.particle_nums").as_int();
    noise_enable_ = node->get_parameter("particle_filter.noise_enable").as_bool();
    foot_align_odom_scale_ = node->get_parameter("particle_filter.foot_align_odom_scale").as_double();
    pt.resize(particle_num);
    soccer_mode_ = node->get_parameter("setup_mode.Soccer_mode").as_bool();

    // ===== Learned (ONNX) localization =====
    node->declare_parameter("learned.mlp_enable", false);
    node->declare_parameter("learned.reloc_enable", false);
    node->declare_parameter("learned.mlp_model", "");
    node->declare_parameter("learned.reloc_model", "");
    node->declare_parameter("learned.reloc_topk", 30);
    node->declare_parameter("learned.sign_right", -1.0);
    node->declare_parameter("learned.sign_forward", 1.0);

    learned_mlp_enable_ = node->get_parameter("learned.mlp_enable").as_bool();
    learned_reloc_enable_ = node->get_parameter("learned.reloc_enable").as_bool();
    learned_mlp_model_ = node->get_parameter("learned.mlp_model").as_string();
    learned_reloc_model_ = node->get_parameter("learned.reloc_model").as_string();
    learned_reloc_topk_ = node->get_parameter("learned.reloc_topk").as_int();
    learned_sign_right_ = node->get_parameter("learned.sign_right").as_double();
    learned_sign_forward_ = node->get_parameter("learned.sign_forward").as_double();
}