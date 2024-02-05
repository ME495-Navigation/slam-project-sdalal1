#include <chrono>
#include <functional>
#include <memory>

#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/control.hpp"

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
  
    control_srv = 
    create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));

    reverse_srv =
    create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    stop_srv =
    create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
  
    cmd_vel_pub = 
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;

  geometry_msgs::msg::Twist vel_pub = geometry_msgs::msg::Twist();

  bool checker = false;

  /// \brief Creates a timer to publish frames and timestep
  void timer_callback()
  {
    if(checker){
    // RCLCPP_INFO_STREAM(this->get_logger(), "in circle timer callback");

    cmd_vel_pub -> publish(vel_pub);
    }
  }

  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    const std::shared_ptr<nuturtle_control::srv::Control::Response>){
    RCLCPP_INFO_STREAM(this->get_logger(), "why am I here bfore serv call????");
    
    checker = true;
    auto lin_velo = request->velocity;
    auto radius = request->radius;
    vel_pub.linear.x = lin_velo;
    // vel_pub.angular.z = lin_velo/radius;
    vel_pub.angular.z = radius;

  }

  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Reversing");
    vel_pub.linear.x *= -1;
    vel_pub.angular.z *= -1;
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping");
    vel_pub.linear.x = 0.0;
    vel_pub.angular.z = 0.0;
    cmd_vel_pub -> publish(vel_pub);
    checker = false;
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