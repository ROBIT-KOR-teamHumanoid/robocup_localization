#include "gui/qnode.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
/// 모델 경로 해석.
///   - 절대경로("/...")면 그대로 쓴다.
///   - "~/" 로 시작하면 홈 디렉터리를 붙인다.
///   - 그 외 상대경로는 이 패키지의 share 디렉터리 기준이다.
std::string resolveModelPath(const std::string &path)
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
}  // namespace

void QNode::getParams()
{

    node->declare_parameter("robot_feature", "kids");
    node->declare_parameter("field.size", "L");   // "S" | "M" | "L"
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

    // ===== 필드 규격 =====
    // 이후의 모든 좌표 변환·랜드마크가 여기에 의존하므로 가장 먼저 확정한다.
    // 알 수 없는 값이면 L 로 떨어뜨리고 경고 (좌표계가 0 인 채로 도는 것보다 낫다).
    field_size_ = node->get_parameter("field.size").as_string();
    const FieldSpecs::FieldSpec *spec = FieldSpecs::fromName(field_size_);
    if (spec == nullptr)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "[field] 알 수 없는 field.size '%s' — S/M/L 중 하나여야 합니다. L 로 진행합니다.",
                     field_size_.c_str());
        field_size_ = "L";
        spec = FieldSpecs::fromName(field_size_);
    }
    FieldCoord::init(*spec);
    MclTune::init(*MclTune::fromName(field_size_));
    // Filter 생성자는 이 시점보다 먼저 돌았으므로 랜드마크를 다시 계산해야 한다.
    Likelihood.filter.rebuild();
    RCLCPP_INFO(node->get_logger(),
                "[field] size=%s  %.1fm x %.1fm  PX_PER_M=%.3f  "
                "obs_filter=%.2fm  max_jump=%.2fm",
                field_size_.c_str(), FieldCoord::A, FieldCoord::B,
                FieldCoord::PX_PER_M, MclTune::OBS_FILTER_DIST_M,
                MclTune::MAX_JUMP_M);

    weight_update_threshold = node->get_parameter("particle_filter.weight_update_threshold").as_double();
    particle_num = node->get_parameter("particle_filter.particle_nums").as_int();
    noise_enable_ = node->get_parameter("particle_filter.noise_enable").as_bool();
    foot_align_odom_scale_ = node->get_parameter("particle_filter.foot_align_odom_scale").as_double();
    motion_.config.noise_enable = noise_enable_;
    motion_.config.foot_align_odom_scale = foot_align_odom_scale_;
    pt.resize(particle_num);
    soccer_mode_ = node->get_parameter("setup_mode.Soccer_mode").as_bool();

    // ===== Learned (ONNX) localization =====
    node->declare_parameter("learned.mlp_enable", false);
    node->declare_parameter("learned.reloc_enable", false);
    // 모델 경로는 절대·상대 둘 다 받는다. 상대경로는 패키지 share 기준이고,
    // 모델은 CMakeLists 의 install(DIRECTORY models ...) 로 거기 놓인다.
    node->declare_parameter("learned.mlp_model", "");
    node->declare_parameter("learned.reloc_model", "");
    node->declare_parameter("learned.reloc_topk", 30);
    node->declare_parameter("learned.sign_right", -1.0);
    node->declare_parameter("learned.sign_forward", 1.0);

    learned_mlp_enable_ = node->get_parameter("learned.mlp_enable").as_bool();
    learned_reloc_enable_ = node->get_parameter("learned.reloc_enable").as_bool();

    learned_mlp_model_ =
        resolveModelPath(node->get_parameter("learned.mlp_model").as_string());
    learned_reloc_model_ =
        resolveModelPath(node->get_parameter("learned.reloc_model").as_string());

    // 없는 파일은 여기서 알려준다. OnnxLocalizer 까지 가면 세션 로드 예외로만
    // 드러나서 "경로가 틀렸다"는 게 잘 안 보인다.
    for (const auto &[enabled, path, name] :
         {std::tuple{learned_mlp_enable_, learned_mlp_model_, "mlp"},
          std::tuple{learned_reloc_enable_, learned_reloc_model_, "reloc"}})
    {
        if (enabled && !std::filesystem::exists(path))
        {
            RCLCPP_ERROR(node->get_logger(),
                         "[learned] %s 모델을 찾을 수 없습니다: %s", name, path.c_str());
        }
    }
    learned_reloc_topk_ = node->get_parameter("learned.reloc_topk").as_int();
    learned_sign_right_ = node->get_parameter("learned.sign_right").as_double();
    learned_sign_forward_ = node->get_parameter("learned.sign_forward").as_double();
}