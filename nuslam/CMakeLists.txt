cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlelib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
add_executable(slam src/slam.cpp)
ament_target_dependencies(slam turtlelib rclcpp std_srvs
nuturtle_control geometry_msgs sensor_msgs nav_msgs nuturtlebot_msgs tf2 tf2_ros visualization_msgs)
target_link_libraries(slam turtlelib::turtlelib "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks turtlelib rclcpp std_srvs nav_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros nuturtlebot_msgs nuturtle_control)  
target_link_libraries(landmarks turtlelib::turtlelib "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

install(TARGETS
  slam
  landmarks
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen src/ README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

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

install(DIRECTORY src launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
