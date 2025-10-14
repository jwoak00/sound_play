cmake_minimum_required(VERSION 3.8)
project(sound_play_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

add_executable(sound_play src/sound_play_node.cpp)
ament_target_dependencies(sound_play rclcpp std_msgs ament_index_cpp tier4_planning_msgs)

target_compile_features(sound_play PUBLIC c_std_99 cxx_std_17)

install(TARGETS sound_play
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch sound
  DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource/
    DESTINATION share/${PROJECT_NAME}/resource)
endif()

ament_package()
