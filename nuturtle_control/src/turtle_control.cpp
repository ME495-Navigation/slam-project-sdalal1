#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/u_int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using turtlelib::almost_equal;

// wheel_radius: 0.033
// track_width: 0.16
// motor_cmd_max: 265
// motor_cmd_per_rad_sec: 0.024
// encoder_ticks_per_rad: 0.00153398078
// collision_radius: 0.11

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", wheel_radius_);
    declare_parameter("track_width", track_width_);
    declare_parameter("motor_cmd_max", motor_cmd_max_);
    declare_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad_);
    declare_parameter("collision_radius", collision_radius_);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    timer_ = create_wall_timer(
      1000ms, std::bind(&TurtleControl::timer_callback, this));

    if(wheel_radius_ == 0.0 || track_width_ ==0.0 || motor_cmd_max_ ==0.0|| encoder_ticks_per_rad_ == 0.0|| collision_radius_ ==0.0){
      RCLCPP_ERROR_STREAM(this->get_logger(), "a paramter not defined not defined");
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius"<<wheel_radius_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width"<<track_width_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_max"<<motor_cmd_max_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad"<<encoder_ticks_per_rad_);
      RCLCPP_ERROR_STREAM(this->get_logger(), "collision_radius"<<collision_radius_);
      // throw std::  find a way to kill it
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  double motor_cmd_max_ = 0.0;
  double encoder_ticks_per_rad_ = 0.0;
  double collision_radius_ = 0.0;

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing:");
  }

};

/// \brief The main function to spin the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;

}