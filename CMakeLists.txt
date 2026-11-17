cmake_minimum_required(VERSION 3.5)
project(robocup_localization)

# colcon passes no build type by default, which builds unoptimized (MCL weighting ~2x slower).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(gamecontroller REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# onnxruntime is searched, not hardcoded: standard paths, CMAKE_PREFIX_PATH, $ONNXRUNTIME_ROOT, $HOME.
option(WITH_INFERENCE "학습형 ONNX 추론 빌드" ON)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ENV ONNXRUNTIME_ROOT ENV HOME
  PATH_SUFFIXES include onnxruntime/include)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  HINTS ENV ONNXRUNTIME_ROOT ENV HOME
  PATH_SUFFIXES lib onnxruntime/lib)
if(WITH_INFERENCE AND NOT (ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY))
  message(WARNING
    "onnxruntime C++ 헤더/라이브러리를 찾을 수 없습니다.\n"
    "  학습형 추론을 끄고 빌드합니다. "
    "사용하려면 ONNXRUNTIME_ROOT 환경변수를 설정하거나 "
    "--cmake-args -DCMAKE_PREFIX_PATH=<onnxruntime 경로> 를 지정하세요.")
  set(WITH_INFERENCE OFF)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobocupLocalization.msg"
)
ament_export_dependencies(rosidl_default_runtime)

find_package(
  rclcpp
  REQUIRED
)
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB PRO_SOURCE_DIR
  RELATIVE ${CMAKE_SOURCE_DIR}
  FOLLOW_SYMLINKS
  CONFIGURE_DEPENDS
  src/*.cpp
  src/core/*.cpp
  src/core/mcl/*.cpp
  src/app/*.cpp
  src/inference/*.cpp
  src/ros/*.cpp
  src/ui/*.cpp
)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR
  RELATIVE ${CMAKE_SOURCE_DIR}
  FOLLOW_SYMLINKS
  include/${PROJECT_NAME}/*.h
  include/${PROJECT_NAME}/*.hpp
  include/${PROJECT_NAME}/ros/*.hpp
  include/${PROJECT_NAME}/ui/*.hpp
)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})
include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}   # for robot_config.h
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

# User-saved files go to the source tree (install/ is recopied every build);
# the executable cannot know the source path, so it is baked in here.
target_compile_definitions(${EXE_TARGET} PRIVATE
    ROBOCUP_LOCALIZATION_SOURCE_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")
ament_target_dependencies(${EXE_TARGET} rclcpp Boost humanoid_interfaces gamecontroller vision_interfaces geometry_msgs OpenCV ament_index_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${EXE_TARGET} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")

# Particle weights are computed in parallel when OpenMP is available (OMP_NUM_THREADS caps it).
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${EXE_TARGET} OpenMP::OpenMP_CXX)
endif()

if(WITH_INFERENCE)
  target_include_directories(${EXE_TARGET} PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")
  target_link_libraries(${EXE_TARGET} "${ONNXRUNTIME_LIBRARY}")
  target_compile_definitions(${EXE_TARGET} PRIVATE ROBOCUP_LOCALIZATION_WITH_ONNX)
  set_target_properties(${EXE_TARGET} PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

install(TARGETS ${EXE_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
