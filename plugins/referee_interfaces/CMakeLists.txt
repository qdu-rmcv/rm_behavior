cmake_minimum_required(VERSION 3.8)
project(referee_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasicHp.msg"
  "msg/Buff.msg"
  "msg/EnemyStatus.msg"
  "msg/AllyBot.msg"
  "msg/GameStatus.msg"
  "msg/Rfid.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()