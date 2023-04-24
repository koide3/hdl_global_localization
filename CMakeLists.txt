cmake_minimum_required(VERSION 3.15)
project(hdl_global_localization)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(thirdparty/json)

################################################
## Declare ROS messages, services and actions ##
################################################

# add_service_files(
#   FILES
#   SetGlobalLocalizationEngine.srv
#   SetGlobalMap.srv
#   QueryGlobalLocalization.srv
# )
# generate_messages(
#   DEPENDENCIES std_msgs sensor_msgs geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryGlobalLocalization.srv"
  "srv/SetGlobalLocalizationEngine.srv"
  "srv/SetGlobalMap.srv"
  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

ament_auto_add_library(hdl_global_localization_engines SHARED
  src/hdl_global_localization/util/config.cpp
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp
)
target_include_directories(hdl_global_localization_engines PUBLIC
  include
  thirdparty/json/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(hdl_global_localization_engines
  ${OpenCV_LIBS}
  spdlog::spdlog
)


ament_auto_add_executable(hdl_global_localization_node
  src/hdl_global_localization_node_ros2.cpp
)
target_link_libraries(hdl_global_localization_node
  hdl_global_localization_engines
)
rosidl_target_interfaces(hdl_global_localization_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_auto_add_executable(hdl_global_localization_test
  src/hdl_global_localization_test_ros2.cpp
)
target_link_libraries(hdl_global_localization_test
  hdl_global_localization_engines
)
rosidl_target_interfaces(hdl_global_localization_test ${PROJECT_NAME} "rosidl_typesupport_cpp")



install(DIRECTORY config DESTINATION share/hdl_global_localization)

ament_auto_package()