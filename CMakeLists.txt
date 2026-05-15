cmake_minimum_required(VERSION 3.8)
project(lidar_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencias ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Biblioteca compartida de strategies ──────────────────────────────────────
# Agrupa la lógica de transformación en una lib interna reutilizable por
# ambos executables (sim y real) sin duplicar código ni linkear dos veces.
add_library(transform_strategies STATIC
  src/manual_transform.cpp
  src/tf2_transform.cpp
)

target_include_directories(transform_strategies PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(transform_strategies PUBLIC cxx_std_17)

ament_target_dependencies(transform_strategies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  robot_interfaces
)

# ── Ejecutable: simulación (ManualTransform) ──────────────────────────────────
add_executable(obstacle_extractor_sim
  src/sim_node_main.cpp
  src/obstacle_extractor_node.cpp
)
target_include_directories(obstacle_extractor_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(obstacle_extractor_sim PUBLIC cxx_std_17)
ament_target_dependencies(obstacle_extractor_sim
  rclcpp sensor_msgs robot_interfaces
)
target_link_libraries(obstacle_extractor_sim transform_strategies)
install(TARGETS obstacle_extractor_sim DESTINATION lib/${PROJECT_NAME})

# ── Ejecutable: robot real (Tf2Transform) ─────────────────────────────────────
add_executable(obstacle_extractor_real
  src/real_node_main.cpp
  src/obstacle_extractor_node.cpp
)
target_include_directories(obstacle_extractor_real PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(obstacle_extractor_real PUBLIC cxx_std_17)
ament_target_dependencies(obstacle_extractor_real
  rclcpp sensor_msgs robot_interfaces
)
target_link_libraries(obstacle_extractor_real transform_strategies)
install(TARGETS obstacle_extractor_real DESTINATION lib/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
