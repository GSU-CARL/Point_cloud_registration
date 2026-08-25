cmake_minimum_required(VERSION 3.8)
project(my_point_reg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters search kdtree features registration)

add_executable(voxel_node src/voxel_node.cpp)
target_include_directories(voxel_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(voxel_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(voxel_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(voxel_node ${PCL_LIBRARIES})

add_executable(feature_node src/feature_node.cpp)
target_include_directories(feature_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(feature_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(feature_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(feature_node ${PCL_LIBRARIES})

add_executable(coarse_registration_node src/coarse_registration_node.cpp)
target_include_directories(coarse_registration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(coarse_registration_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(coarse_registration_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(coarse_registration_node ${PCL_LIBRARIES})

add_executable(fine_registration_node src/fine_registration_node.cpp)
target_include_directories(fine_registration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(fine_registration_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(fine_registration_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(fine_registration_node ${PCL_LIBRARIES})

add_executable(merge_node src/merge_node.cpp)
target_include_directories(merge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(merge_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(merge_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(merge_node ${PCL_LIBRARIES})

add_executable(dedup_node src/dedup_node.cpp)
target_include_directories(dedup_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(dedup_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dedup_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(dedup_node ${PCL_LIBRARIES})

add_executable(pipeline_node src/pipeline_node.cpp)
target_include_directories(pipeline_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(pipeline_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pipeline_node
  rclcpp
  std_srvs
  pcl_ros
  pcl_conversions
)
target_link_libraries(pipeline_node ${PCL_LIBRARIES})

install(TARGETS voxel_node feature_node coarse_registration_node
  fine_registration_node merge_node dedup_node pipeline_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
