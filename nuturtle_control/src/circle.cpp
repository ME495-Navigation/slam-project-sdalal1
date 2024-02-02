#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/"

using namespace std::chrono_literals;

/// \brief Node to start simulation in RVIZ
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    declare_parameter("rate", 100.0);

    auto rate =  get_parameter("rate").as_double();

    timer_ = create_wall_timer(
      1000ms / rate, std::bind(&Circle::timer_callback, this));
  
    
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // rclcpp::Service<nuturtle_control::srv::
  geometry_msgs::msg::Twist vel_pub = geometry_msgs::msg::Twist();

  bool checker = false;

  /// \brief Creates a timer to publish frames and timestep
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing:");
  }
};
/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;

}