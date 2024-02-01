#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("body_id", "base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "left_wheel");
    declare_parameter("wheel_right", "right_right");

    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();

    joint_subscription = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Odometry::joint_callback, this, std::placeholders::_1));


  }
private:
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        
    }
};