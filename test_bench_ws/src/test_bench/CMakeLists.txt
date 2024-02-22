cmake_minimum_required(VERSION 3.8)
project(test_bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Below are all package dependencies for the test bench,
# add to this list as needed
find_package(ament_cmake REQUIRED)
find_package(spinnaker_camera_driver REQUIRED)
find_package(sick_scan_xd REQUIRED)
# find_package(<dependency> REQUIRED)

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

##! Here you define the source and destination dirs for project install
install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()