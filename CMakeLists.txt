cmake_minimum_required(VERSION 3.16)
project(rm_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(rm_behavior_server src/rm_behavior_server.cpp)

ament_auto_add_library(action_client SHARED
  src/rm_behavior_client.cpp
)

rclcpp_components_register_node(action_client
  PLUGIN rm_behavior::BehaviorClient
  EXECUTABLE rm_behavior_client
)

####### Conditions #######

###### 场地关键数据判断 ######
# ament_auto_add_library(event_data SHARED plugins/condition/event_data.cpp)
# list(APPEND plugin_libs event_data)

###### 被击打判断 ######
# ament_auto_add_library(hit SHARED plugins/condition/hit.cpp)
# list(APPEND plugin_libs hit)

###### 前哨战与基地血量判断 ######
# ament_auto_add_library(hp_check SHARED plugins/condition/hp_check.cpp)
# list(APPEND plugin_libs hp_check)

# ###### rfid 数据判断 ######
# ament_auto_add_library(rfid SHARED plugins/condition/rfid.cpp)
# list(APPEND plugin_libs rfid)

###### 哨兵状态判断 ######
ament_auto_add_library(status_check SHARED src/plugins/condition/status_check.cpp)
list(APPEND plugin_libs status_check)

# ###### 比赛时间判断 ######
# ament_auto_add_library(time_check SHARED plugins/condition/time_check.cpp)
# list(APPEND plugin_libs time_check)

# ###### 追踪判断 ######
# ament_auto_add_library(tracking SHARED plugins/condition/tracking.cpp)
# list(APPEND plugin_libs tracking)

####### Actions #######
ament_auto_add_library(nav2goal SHARED src/plugins/action/nav2goal.cpp)
list(APPEND plugin_libs nav2goal)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  launch
  params
)
