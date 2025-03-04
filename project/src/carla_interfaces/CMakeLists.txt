cmake_minimum_required(VERSION 3.8)
project(carla_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES 
  "msg/CarlaAudio.msg"
  "msg/CarlaCameraControl.msg"
  "msg/CarlaMultirotorControl.msg"
  "msg/VehicleStatus.msg"
  "msg/SetActorOdometry.msg"
  "msg/SetActorPhysics.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSyncState.srv"
  ${MSG_FILES}
  
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

foreach(MSG ${MSG_FILES})
  get_filename_component(FILEDIR "${MSG}" DIRECTORY)
  get_filename_component(FILENAME "${MSG}" NAME_WE)

  add_custom_command(
    TARGET carla_interfaces POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${PROJECT_NAME}/${FILEDIR}/${FILENAME}.idl
      ${CMAKE_SOURCE_DIR}/idl/${PROJECT_NAME}/${FILEDIR}/${FILENAME}.idl)

endforeach()