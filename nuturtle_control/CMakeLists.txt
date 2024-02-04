cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

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
find_package(rosidl_default_generators REQUIRED)
find_package(catch_ros2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control turtlelib rclcpp std_msgs std_srvs geometry_msgs sensor_msgs nuturtlebot_msgs)
target_link_libraries(turtle_control turtlelib::turtlelib)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry turtlelib rclcpp std_srvs geometry_msgs sensor_msgs nav_msgs nuturtlebot_msgs tf2 tf2_ros)
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(circle src/circle.cpp)
ament_target_dependencies(circle turtlelib rclcpp std_srvs geometry_msgs)
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(turtle_control_test test/turtle_control_test.cpp)
ament_target_dependencies(turtle_control_test rclcpp geometry_msgs nuturtlebot_msgs sensor_msgs)
target_link_libraries(turtle_control_test catch_ros2::catch_ros2_with_node_main "${cpp_typesupport_target}")

add_executable(turtle_odom_test test/turtle_odom_test.cpp)
ament_target_dependencies(turtle_odom_test turtlelib rclcpp std_srvs geometry_msgs sensor_msgs nav_msgs nuturtlebot_msgs tf2 tf2_ros)
target_link_libraries(turtle_odom_test catch_ros2::catch_ros2_with_node_main "${cpp_typesupport_target}")


install(TARGETS
  turtle_control
  odometry
  circle
  turtle_control_test
  turtle_odom_test
  DESTINATION lib/${PROJECT_NAME})

install(FILES
test/turtle_control_test.launch.xml
test/turtle_odom_test.launch.xml
DESTINATION share/${PROJECT_NAME}
)

catch_ros2_add_integration_test(test_1
  LAUNCH_FILE turtle_control_test.launch.xml 
)

catch_ros2_add_integration_test(test_2
  LAUNCH_FILE turtle_odom_test.launch.xml 
)
find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

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

install(DIRECTORY src srv launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
