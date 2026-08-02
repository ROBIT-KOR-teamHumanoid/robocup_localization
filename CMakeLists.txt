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
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp src/filter/*.cpp src/get_params/*.cpp src/robot_odom_model/*.cpp src/udp/*.cpp src/onnx_infer/*.cpp)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
list(FILTER PRO_INCLUDE_DIR EXCLUDE REGEX "filter\\.hpp$")
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
