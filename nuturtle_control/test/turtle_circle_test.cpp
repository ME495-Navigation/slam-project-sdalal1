#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;

geometry_msgs::msg::Twist twi = geometry_msgs::msg::Twist();

double count = 0;
void velo_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  msg->angular.x;
  msg->angular.z;
  count++;
}

TEST_CASE("Circular subscription test", "[sub_freq]") {
  auto node = rclcpp::Node::make_shared("test_circle");

  auto vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, &velo_callback);

  auto client = node->create_client<nuturtle_control::srv::Control>("control");

  auto request = std::make_shared<nuturtle_control::srv::Control::Request>();
  request->velocity = 0.1;
  request->radius = 0.1;

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (!client->wait_for_service(0s)) {

    if (!rclcpp::ok()) {
      std::cout << "waiting on service" << std::endl;
    }
  }
  auto future_result = client->async_send_request(request);

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 3s)
  )
  {
    // Repeatedly check for the dummy service until its found
    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK_THAT(count, Catch::Matchers::WithinAbs(300, 20));
};
