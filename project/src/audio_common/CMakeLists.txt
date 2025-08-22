cmake_minimum_required(VERSION 3.8)
project(audio_common)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(carla_interfaces REQUIRED)
find_library(PORTAUDIO_LIB portaudio REQUIRED)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
)

# audio_player_node
add_executable(audio_player_node
  src/audio_common/audio_player_node.cpp
  src/audio_player_main.cpp
)
ament_target_dependencies(audio_player_node rclcpp carla_interfaces)
target_link_libraries(audio_player_node ${PORTAUDIO_LIB})
install(TARGETS
  audio_player_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
