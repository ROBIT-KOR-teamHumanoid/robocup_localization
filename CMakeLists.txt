cmake_minimum_required(VERSION 3.5)
project(robocup_localization)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(gamecontroller REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# onnxruntime (C++) — 학습형 ONNX 추론
set(ONNXRUNTIME_ROOTDIR "/home/bws/onnxruntime" CACHE PATH "onnxruntime C++ install root")
if(NOT EXISTS "${ONNXRUNTIME_ROOTDIR}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "onnxruntime C++ 헤더를 찾을 수 없습니다: ${ONNXRUNTIME_ROOTDIR}/include (ONNXRUNTIME_ROOTDIR 확인)")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobocupLocalization.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(
  rclcpp
  REQUIRED
)
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)
# ONNX 모델은 배포되는 런타임 자산이라 패키지 데이터로 함께 설치한다.
# config 의 learned.mlp_model 은 이 share 디렉터리 기준 상대경로다.
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# GLOB 대신 명시 목록. GLOB 은 파일을 추가·이동해도 CMake 재실행 전까지
# 조용히 무시되어, "왜 수정이 반영 안 되지" 로 나타난다.
set(PRO_SOURCE_DIR
  src/main.cpp
  src/gui/qnode.cpp
  src/gui/main_window.cpp
  src/gui/get_params.cpp
  src/gui/odom_param_store.cpp
  src/field_profile/field_coordinate.cpp
  src/monte_carlo_localization/filter.cpp
  src/monte_carlo_localization/mcl_node.cpp
  src/monte_carlo_localization/motion_model.cpp
  src/monte_carlo_localization/particle_reset.cpp
  src/pre_processing/vision_processing.cpp
  src/pre_processing/ball_tracker.cpp
  src/monte_carlo_localization/mcl_engine.cpp
  src/mlp_localization/mlp_onnx_infer.cpp
  src/utils/team_udp_receiver.cpp
  src/utils/yaw_compensator.cpp
)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
# moc 대상 = Q_OBJECT 를 가진 헤더만. 빠뜨리면 컴파일은 되고 링크에서
# undefined reference to vtable 로 터진다.
set(PRO_INCLUDE_DIR
  include/gui/qnode.hpp
  include/gui/main_window.hpp
)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})
include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}   # robot_config.h 검색 경로
	${ONNXRUNTIME_ROOTDIR}/include
)
set(EXE_TARGET ${PROJECT_NAME}_bin)
add_executable(${EXE_TARGET}
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${QT_MOC_HPP}
    ${QT_RESOUCE_HPP}
    ${QT_UI_HPP}
  )
set_target_properties(${EXE_TARGET} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
ament_target_dependencies(${EXE_TARGET} rclcpp Boost humanoid_interfaces gamecontroller vision_interfaces OpenCV ament_index_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${EXE_TARGET} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network ${OpenCV_LIBRARIES} "${cpp_typesupport_target}" "${ONNXRUNTIME_ROOTDIR}/lib/libonnxruntime.so")

# libonnxruntime.so.1 런타임 검색 경로 (RPATH)
set_target_properties(${EXE_TARGET} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOTDIR}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOTDIR}/lib")
install(TARGETS ${EXE_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
